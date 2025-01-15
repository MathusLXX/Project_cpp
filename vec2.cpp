#include "vec2.h"
#include <cmath>
Vec2::Vec2(float x_, float y_):x(x_), y(y_) {}

float Vec2::get_x() const{
    return x;
}

float Vec2::get_y() const{
    return y;
}

float Vec2::dot(Vec2 other){
    return x*other.get_y() + y*other.get_x();
}

Vec2 Vec2::rotate(float angle) const {
    float cos_theta = std::cos(angle);
    float sin_theta = std::sin(angle);
    return Vec2(
        x * cos_theta - y * sin_theta,
        x * sin_theta + y * cos_theta
        );
}

float Vec2::length() const {
    return std::sqrt(x * x + y * y);
}

Vec2 Vec2::normalized() const {
    float len = length();
    return (len > 0) ? (*this * (1.0f / len)) : Vec2(0, 0);
}
