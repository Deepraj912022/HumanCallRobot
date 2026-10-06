#include "behavior/RobotStateMachine.hpp"
#include <iostream>

namespace human_call_robot {

RobotStateMachine::RobotStateMachine() {
    current_state_.mode = RobotMode::IDLE;
}

RobotMode RobotStateMachine::getCurrentMode() const {
    SpinLockGuard lock(state_mutex_);
    return current_state_.mode;
}

RobotState RobotStateMachine::getRobotState() const {
    SpinLockGuard lock(state_mutex_);
    return current_state_;
}

void RobotStateMachine::updatePose(const Pose2D& pose) {
    SpinLockGuard lock(state_mutex_);
    current_state_.pose = pose;
}

void RobotStateMachine::updateVelocity(const Twist& velocity) {
    SpinLockGuard lock(state_mutex_);
    current_state_.velocity = velocity;
}

void RobotStateMachine::updateBattery(double battery_pct) {
    SpinLockGuard lock(state_mutex_);
    current_state_.battery_level = battery_pct;
}

void RobotStateMachine::transitionTo(RobotMode new_mode) {
    RobotMode old_mode;
    StateChangeCallback cb_to_call = nullptr;

    {
        SpinLockGuard lock(state_mutex_);
        if (current_state_.emergency_stopped && new_mode != RobotMode::EMERGENCY_STOP) {
            return; // Cannot exit emergency stop without explicit reset
        }

        if (current_state_.mode == new_mode) return;

        old_mode = current_state_.mode;
        current_state_.mode = new_mode;
        cb_to_call = state_change_cb_;
    }

    if (cb_to_call) {
        cb_to_call(old_mode, new_mode);
    }
}

void RobotStateMachine::triggerEmergencyStop() {
    RobotMode old_mode;
    StateChangeCallback cb_to_call = nullptr;

    {
        SpinLockGuard lock(state_mutex_);
        old_mode = current_state_.mode;
        current_state_.mode = RobotMode::EMERGENCY_STOP;
        current_state_.emergency_stopped = true;
        current_state_.velocity = Twist(0.0, 0.0, 0.0);
        cb_to_call = state_change_cb_;
    }

    if (cb_to_call && old_mode != RobotMode::EMERGENCY_STOP) {
        cb_to_call(old_mode, RobotMode::EMERGENCY_STOP);
    }
}

void RobotStateMachine::resetEmergencyStop() {
    SpinLockGuard lock(state_mutex_);
    current_state_.emergency_stopped = false;
    current_state_.mode = RobotMode::IDLE;
}

void RobotStateMachine::setTarget(const TargetState& target) {
    SpinLockGuard lock(state_mutex_);
    current_target_ = target;
}

TargetState RobotStateMachine::getTarget() const {
    SpinLockGuard lock(state_mutex_);
    return current_target_;
}

void RobotStateMachine::setStateChangeCallback(StateChangeCallback callback) {
    SpinLockGuard lock(state_mutex_);
    state_change_cb_ = callback;
}

} // namespace human_call_robot
