#ifndef CONTEXT_H
#define CONTEXT_H

#include <vector>
#include <memory>
#include <QPainter>
#include <QPaintEvent>
#include <QPointF>
#include "vec2.h"
#include "collider.h"

class Bar {
private:
    Vec2 rotate(const Vec2& point, float angle) const;

public:
    Bar(const Vec2& center, float width, float height, float angle);
    std::vector<PlaneCollider> colliders;
    Vec2 topRight;
    Vec2 topLeft ;
    Vec2 bottomRight;
    Vec2 bottomLeft;
};

class Context {
public:
    Context();

    void addParticle(const Vec2& position, const Vec2& velocity, float radius, float mass);
    void addStaticContactConstraints();
    void display(QPainter* painter, QPaintEvent* event) const;
    void updatePhysicalSystem(float dt);

private:
    std::vector<std::shared_ptr<Particle>> particles;
    std::vector<Bar> bars;
    std::vector<SphereCollider> spheres;
    void applyExternalForce(float dt);
    void dampVelocities(float dt);
    void updateExpectedPosition(float dt);
    void addDynamicContactConstraints(float dt);
    void addStaticContactConstraints(float dt);
    void enforceBoundaryConstraints(float width, float height);
};

#endif // CONTEXT_H
