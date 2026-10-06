#ifndef HUMAN_CALL_ROBOT_CORE_VECTOR2D_HPP_
#define HUMAN_CALL_ROBOT_CORE_VECTOR2D_HPP_

#include <cmath>

namespace human_call_robot {

class Vector2D {
public:
    double x;
    double y;

    Vector2D() : x(0.0), y(0.0) {}
    Vector2D(double x_val, double y_val) : x(x_val), y(y_val) {}

    double length() const {
        return std::sqrt(x * x + y * y);
    }

    double lengthSquared() const {
        return x * x + y * y;
    }

    Vector2D normalized() const {
        double len = length();
        if (len < 1e-9) return Vector2D(0.0, 0.0);
        return Vector2D(x / len, y / len);
    }

    double dot(const Vector2D& other) const {
        return x * other.x + y * other.y;
    }

    double distanceTo(const Vector2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }

    Vector2D operator*(double scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }

    Vector2D operator/(double scalar) const {
        return Vector2D(x / scalar, y / scalar);
    }
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_VECTOR2D_HPP_
