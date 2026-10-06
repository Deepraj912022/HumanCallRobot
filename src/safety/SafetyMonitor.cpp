#include "safety/SafetyMonitor.hpp"
#include <algorithm>

namespace human_call_robot {

template<typename T>
static T clampVal(T val, T min_val, T max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

SafetyMonitor::SafetyMonitor(double min_obstacle_dist, double max_linear_vel)
    : min_obstacle_dist_(min_obstacle_dist), max_linear_vel_(max_linear_vel) {}

void SafetyMonitor::setMinObstacleDistance(double dist) {
    min_obstacle_dist_ = dist;
}

bool SafetyMonitor::checkSafety(double closest_obstacle_dist, const Twist& requested_vel, Twist& safe_vel, std::string& out_reason) {
    safe_vel = requested_vel;

    // Check emergency obstacle proximity
    if (closest_obstacle_dist < min_obstacle_dist_) {
        safe_vel.linear_x = 0.0;
        safe_vel.linear_y = 0.0;
        out_reason = "Emergency Stop: Obstacle detected too close (" + std::to_string(closest_obstacle_dist) + "m)";
        return false;
    }

    // Slow down if approaching obstacle
    if (closest_obstacle_dist < min_obstacle_dist_ * 2.0 && requested_vel.linear_x > 0) {
        double scale = (closest_obstacle_dist - min_obstacle_dist_) / min_obstacle_dist_;
        safe_vel.linear_x = requested_vel.linear_x * clampVal(scale, 0.1, 1.0);
    }

    // Enforce max velocity limits
    safe_vel.linear_x = clampVal(safe_vel.linear_x, -max_linear_vel_, max_linear_vel_);

    out_reason = "OK";
    return true;
}

} // namespace human_call_robot
