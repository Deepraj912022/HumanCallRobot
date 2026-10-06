#ifndef HUMAN_CALL_ROBOT_CORE_TARGET_STATE_HPP_
#define HUMAN_CALL_ROBOT_CORE_TARGET_STATE_HPP_

#include "Pose2D.hpp"
#include <string>
#include <cstdint>

namespace human_call_robot {

struct TargetState {
    std::string target_id;
    std::string location_name;
    Pose2D pose;
    double confidence{1.0}; // 0.0 to 1.0 (for person detection)
    bool is_person{false};
    uint64_t timestamp_ms{0};
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_TARGET_STATE_HPP_
