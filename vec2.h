#ifndef VEC2_H
#define VEC2_H

class Vec2
{
public:
    Vec2(float x, float y);
    ~Vec2() = default;
    float get_x() const;
    float get_y() const;

    float dot(Vec2 other);
    Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }
    Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }
    Vec2 operator*(float scalar) const { return {x * scalar, y * scalar}; }
    Vec2& operator+=(const Vec2& other) { x += other.x; y += other.y; return *this; }
    Vec2& operator-=(const Vec2& other) { x -= other.x; y -= other.y; return *this; }
    Vec2 normalized() const;
    float length() const;
    Vec2 rotate(float angle) const;
private:
    float x;
    float y;
};

#endif // VEC2_H
