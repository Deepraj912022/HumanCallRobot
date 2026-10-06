#ifndef HUMAN_CALL_ROBOT_CORE_POSE2D_HPP_
#define HUMAN_CALL_ROBOT_CORE_POSE2D_HPP_

#include "Vector2D.hpp"
#include <cmath>

namespace human_call_robot {

class Pose2D {
public:
    double x;
    double y;
    double theta; // Orientation angle in radians [-pi, pi]

    Pose2D() : x(0.0), y(0.0), theta(0.0) {}
    Pose2D(double x_val, double y_val, double theta_val)
        : x(x_val), y(y_val), theta(normalizeAngle(theta_val)) {}
    Pose2D(const Vector2D& pos, double theta_val)
        : x(pos.x), y(pos.y), theta(normalizeAngle(theta_val)) {}

    Vector2D position() const {
        return Vector2D(x, y);
    }

    double distanceTo(const Pose2D& other) const {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_POSE2D_HPP_
