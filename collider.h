#ifndef COLLIDER_H
#define COLLIDER_H

#include <QMainWindow>
#include <QObject>
#include <QWidget>
#include <iostream>
#include "vec2.h"

struct Particle {
    Vec2 pos;
    Vec2 velocity;
    float radius;
    float mass;

    Particle(Vec2 p, Vec2 v, float r, float m) : pos(p), velocity(v), radius(r), mass(m) {}
};

struct StaticConstraint {
    Vec2 normal;
    float penetration;
};

class Collider {
public:
    virtual std::optional<StaticConstraint> checkContact(const Particle& particle) const = 0;
};


class PlaneCollider : public Collider {
private:
    const float size;

public:

    const Vec2 start;
    const Vec2 end;
    Vec2 normal;
    PlaneCollider(const Vec2& _begin, const Vec2& _end);

    //nous cherchions une solution pour éviter des fuites de mémoire et nous avont trouvé ce type sur internet qui permet de renvoyer soit une valeur
    //soit null sans risque de fuites de mémoires
    std::optional<StaticConstraint> checkContact(const Particle& particle) const override;
};

class SphereCollider : public Collider {


public:
    SphereCollider(const Vec2& center, float radius) : center(center), radius(radius) {}
    Vec2 center;
    float radius;
    std::optional<StaticConstraint> checkContact(const Particle& particle) const override;
};

#endif // COLLIDER_H
