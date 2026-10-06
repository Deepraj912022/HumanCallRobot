#ifndef HUMAN_CALL_ROBOT_CORE_ROBOT_STATE_HPP_
#define HUMAN_CALL_ROBOT_CORE_ROBOT_STATE_HPP_

#include "Pose2D.hpp"
#include "Twist.hpp"
#include "Types.hpp"
#include <cstdint>

namespace human_call_robot {

struct RobotState {
    Pose2D pose;
    Twist velocity;
    RobotMode mode{RobotMode::IDLE};
    double battery_level{100.0}; // Percentage
    bool emergency_stopped{false};
    uint64_t timestamp_ms{0};
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_ROBOT_STATE_HPP_
