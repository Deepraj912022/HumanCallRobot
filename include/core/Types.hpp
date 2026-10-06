#ifndef HUMAN_CALL_ROBOT_CORE_TYPES_HPP_
#define HUMAN_CALL_ROBOT_CORE_TYPES_HPP_

#include <string>
#include <cstdint>
#include <chrono>

namespace human_call_robot {

enum class RobotMode {
    IDLE,
    SEARCHING,
    APPROACHING,
    ARRIVED,
    FOLLOWING,
    EMERGENCY_STOP,
    ERROR
};

inline std::string robotModeToString(RobotMode mode) {
    switch (mode) {
        case RobotMode::IDLE: return "IDLE";
        case RobotMode::SEARCHING: return "SEARCHING";
        case RobotMode::APPROACHING: return "APPROACHING";
        case RobotMode::ARRIVED: return "ARRIVED";
        case RobotMode::FOLLOWING: return "FOLLOWING";
        case RobotMode::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case RobotMode::ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

enum class CallPriority {
    LOW = 0,
    NORMAL = 1,
    HIGH = 2,
    EMERGENCY = 3
};

enum class CallStatus {
    PENDING,
    IN_PROGRESS,
    COMPLETED,
    CANCELLED
};

struct CallRequest {
    std::string call_id;
    std::string location_id;
    CallPriority priority;
    CallStatus status;
    uint64_t timestamp_ms;
    std::string requester_info;
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_TYPES_HPP_
