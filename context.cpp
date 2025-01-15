#include "context.h"
#include "vec2.h"


Bar::Bar(const Vec2& center, float width, float height, float angle) :
    topRight (Vec2 (width / 2.0, height / 2.0) + center),
    topLeft (Vec2(-width / 2, height / 2) + center),
    bottomRight (Vec2(width / 2, -height / 2) + center),
    bottomLeft (Vec2(-width / 2.0, -height / 2.0) + center){
    colliders.emplace_back(topLeft, bottomLeft);
    colliders.emplace_back(topRight, bottomRight);
    colliders.emplace_back(bottomRight, bottomLeft);
    colliders.emplace_back(topLeft, topRight);
    std::cout << topLeft.get_x() << " " << topLeft.get_y() << std::endl;

}

Vec2 Bar::rotate(const Vec2& point, float angle) const {
    float cosA = std::cos(angle);
    float sinA = std::sin(angle);
    return Vec2(point.get_x() * cosA - point.get_y() * sinA, point.get_x() * sinA + point.get_y() * cosA);
}



Context::Context() {
    bars.emplace_back(Vec2(300.0, 300.0), 100.0, 20.0, 0);
}

void Context::addParticle(const Vec2& position, const Vec2& velocity, float radius, float mass) {
    particles.push_back(std::make_shared<Particle>(position, velocity, radius, mass));
}


void Context::display(QPainter* painter, QPaintEvent* event) const {
    painter->fillRect(event->rect(), QBrush(QColor(64, 64, 64)));
    painter->setRenderHint(QPainter::Antialiasing);
    painter->setPen(Qt::NoPen);
    painter->setBrush(Qt::gray);

    for (const auto& bar : bars) {

        QPolygonF polygon;
        polygon << QPointF(bar.topLeft.get_x(), bar.topLeft.get_y())
                << QPointF(bar.bottomLeft.get_x(), bar.bottomLeft.get_y())
                << QPointF(bar.bottomRight.get_x(), bar.bottomRight.get_y())
                << QPointF(bar.topRight.get_x(), bar.topRight.get_y());


        painter->drawPolygon(polygon);
    }


    painter->setPen(Qt::blue);
    painter->setBrush(Qt::red);

    for (const auto& particle : particles) {
        QRectF circle(particle->pos.get_x() - particle->radius,
                      particle->pos.get_y() - particle->radius,
                      2 * particle->radius,
                      2 * particle->radius);
        painter->drawEllipse(circle);
    }
}



void resolveCollision(Particle& particle, const StaticConstraint& constraint) {
    particle.pos = particle.pos + constraint.normal * constraint.penetration;
    float x_velocity;
    particle.velocity = {-particle.velocity.get_x(), -particle.velocity.get_y()};
    Vec2 velocityAlongNormal = constraint.normal * (particle.velocity.get_x() * constraint.normal.get_x() + particle.velocity.get_y() * constraint.normal.get_y());
    particle.velocity = particle.velocity - velocityAlongNormal;
}


void Context::updatePhysicalSystem(float dt) {
    applyExternalForce(dt);
    //dampVelocities(dt);
    addStaticContactConstraints();
    addDynamicContactConstraints(dt);
    enforceBoundaryConstraints(400.0, 400.0);
    updateExpectedPosition(dt);

}


void Context::applyExternalForce(float dt) {
    const Vec2 gravity(0.0f, 9.81f);
    for (auto& particle : particles) {
        particle->velocity = particle->velocity + gravity * dt;
    }
}


void Context::dampVelocities(float dt) {
    const float dampingFactor = 0.9f;
    for (auto& particle : particles) {
        particle->velocity = particle->velocity * dampingFactor;
    }
}


void Context::updateExpectedPosition(float dt) {
    for (auto& particle : particles) {
        particle->pos = particle->pos + particle->velocity * dt;
    }
}


void Context::addDynamicContactConstraints(float dt) {
    // je n'ai pas réussi à introduire des double collisions donc il n'est pas possible
    // d'empiler les particules
    for (size_t i = 0; i < particles.size(); ++i) {
        for (size_t j = i + 1; j < particles.size(); ++j) {
            auto& p1 = particles[i];
            auto& p2 = particles[j];
            Vec2 norm = p2->pos - p1->pos;
            float dist = std::sqrt(norm.get_x() * norm.get_x() + norm.get_y() * norm.get_y());
            float surplus = p1->radius + p2->radius - dist;

            if (surplus > 0) {
                Vec2 coll = norm * (1.0f / dist);

                // Relative velocity
                Vec2 rvit = p2->velocity - p1->velocity;
                float vnormal = rvit.dot(coll);
                // si les particules s'éloignent, on ne fait rien
                if (vnormal > 0) continue;
                //e : coeff d'élasticité : les particules rebondissent plus ou moins entre elles
                float e = 0.1f;
                float scalar = -(1 + e) * vnormal / (1 / p1->mass + 1 / p2->mass);

                Vec2 impulse = coll * scalar;
                p1->velocity -= impulse * 0.6F * (1 / p1->mass);
                p2->velocity += impulse *  0.6F * (1 / p2->mass);

                // correction pour ne pas que les particules se chevauchent et qu'elles finissent exactement au même endroit
                float correction = 0.1f;
                Vec2 corr = coll * (surplus * correction);
                p1->pos -= corr;
                p2->pos += corr;
            }
        }
    }
}




void Context::addStaticContactConstraints() {
    for (auto& particle : particles) {
        for (auto& bar : bars) {
            for (auto& collider : bar.colliders){
                auto constraint = collider.checkContact(*particle);
                if (constraint.has_value()) {
                    resolveCollision(*particle, *constraint);


                }
            }
        }
    }
}





void Context::enforceBoundaryConstraints(float width, float height) {
    for (auto& particle : particles) {

        if (particle->pos.get_x() - particle->radius < 0) {
            particle->pos = Vec2(particle->radius, particle->pos.get_y());
            particle->velocity = Vec2(-particle->velocity.get_x(), particle->velocity.get_y());
        }


        if (particle->pos.get_x() + particle->radius > width) {
            particle->pos = Vec2(width - particle->radius, particle->pos.get_y());
            particle->velocity = Vec2(-particle->velocity.get_x(), particle->velocity.get_y());
        }

        if (particle->pos.get_y() - particle->radius < 0) {
            particle->pos = Vec2(particle->pos.get_x(), particle->radius);
            particle->velocity = Vec2(0.0F, 0.0F);
        }

        if (particle->pos.get_y() + particle->radius > height) {
            particle->pos = Vec2(particle->pos.get_x(), height - particle->radius);
            // empeche que la particule rebondisse à l'infini
            if (particle-> velocity.get_y() < 5.0F){
                particle->velocity = Vec2(0.0F, 0.0F);
            }
            else{
                particle->velocity = Vec2(particle->velocity.get_x()*0.6, -particle->velocity.get_y()*0.6);
            }
        }
    }
}
