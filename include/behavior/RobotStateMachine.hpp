#ifndef HUMAN_CALL_ROBOT_BEHAVIOR_ROBOT_STATE_MACHINE_HPP_
#define HUMAN_CALL_ROBOT_BEHAVIOR_ROBOT_STATE_MACHINE_HPP_

#include "core/Types.hpp"
#include "core/RobotState.hpp"
#include "core/TargetState.hpp"
#include "core/Mutex.hpp"
#include <functional>

namespace human_call_robot {

class RobotStateMachine {
public:
    using StateChangeCallback = std::function<void(RobotMode prev_mode, RobotMode new_mode)>;

    RobotStateMachine();

    RobotMode getCurrentMode() const;
    RobotState getRobotState() const;

    void updatePose(const Pose2D& pose);
    void updateVelocity(const Twist& velocity);
    void updateBattery(double battery_pct);

    void transitionTo(RobotMode new_mode);
    void triggerEmergencyStop();
    void resetEmergencyStop();

    void setTarget(const TargetState& target);
    TargetState getTarget() const;

    void setStateChangeCallback(StateChangeCallback callback);

private:
    mutable SpinMutex state_mutex_;
    RobotState current_state_;
    TargetState current_target_;
    StateChangeCallback state_change_cb_{nullptr};
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_BEHAVIOR_ROBOT_STATE_MACHINE_HPP_
