#ifndef HUMAN_CALL_ROBOT_SAFETY_SAFETY_MONITOR_HPP_
#define HUMAN_CALL_ROBOT_SAFETY_SAFETY_MONITOR_HPP_

#include "core/Twist.hpp"
#include <string>

namespace human_call_robot {

class SafetyMonitor {
public:
    SafetyMonitor(double min_obstacle_dist = 0.3, double max_linear_vel = 0.5);

    bool checkSafety(double closest_obstacle_dist, const Twist& requested_vel, Twist& safe_vel, std::string& out_reason);
    void setMinObstacleDistance(double dist);

private:
    double min_obstacle_dist_{0.3}; // meters
    double max_linear_vel_{0.5};   // m/s
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_SAFETY_SAFETY_MONITOR_HPP_
