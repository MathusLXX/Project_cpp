#include "collider.h"

PlaneCollider::PlaneCollider(const Vec2& _begin, const Vec2& _end) :
    start(_begin), end(_end),normal((start-end).normalized()), size((end-start).length()) {
        normal = Vec2(-normal.get_y(), normal.get_x());
}

PlaneCollider::PlaneCollider(const Vec2& _begin, const Vec2& _end, const Vec2& normalDir) :
    start(_begin), end(_end),normal(normalDir.normalized()), size((end-start).length()) {
    normal = Vec2(-normal.get_y(), normal.get_x());
}
std::optional<StaticConstraint> PlaneCollider::checkContact(const Particle& particle) const {
    Vec2 toParticle = particle.pos - start;
    float projectionLength = std::abs(toParticle.get_x() * normal.get_x() + toParticle.get_y() * normal.get_y());
    Vec2 segmentDirection = (end - start).normalized();
    float segmentLength = (end - start).length();
    float projectionOnSegment = (toParticle.get_x() * segmentDirection.get_x() + toParticle.get_y() * segmentDirection.get_y());

    if (projectionOnSegment < 0 || projectionOnSegment > segmentLength) {
        return std::nullopt;
    }

    if (projectionLength < particle.radius) {
        return StaticConstraint{normal, particle.radius - projectionLength};
    }

    return std::nullopt;
}

std::optional<StaticConstraint> SphereCollider::checkContact(const Particle& particle) const {
    Vec2 delta = particle.pos - center;
    float dist = delta.length();
    if (dist < particle.radius + radius) {
        return StaticConstraint{delta, particle.radius + radius - dist};
    }
    return std::nullopt;
}
