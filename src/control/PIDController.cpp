#include "control/PIDController.hpp"
#include <algorithm>

namespace human_call_robot {

template<typename T>
static T clampVal(T val, T min_val, T max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

PIDController::PIDController() {}

PIDController::PIDController(double kp, double ki, double kd, double min_out, double max_out)
    : kp_(kp), ki_(ki), kd_(kd), min_output_(min_out), max_output_(max_out) {
    min_integral_ = min_out;
    max_integral_ = max_out;
}

void PIDController::setGains(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(double min_out, double max_out) {
    min_output_ = min_out;
    max_output_ = max_out;
}

void PIDController::setIntegralLimits(double min_int, double max_int) {
    min_integral_ = min_int;
    max_integral_ = max_int;
}

double PIDController::compute(double setpoint, double measurement, double dt) {
    if (dt <= 0.0) return 0.0;

    double error = setpoint - measurement;

    // Proportional term
    double p_term = kp_ * error;

    // Integral term with anti-windup clamping
    integral_ += error * dt;
    double i_term = ki_ * integral_;
    i_term = clampVal(i_term, min_integral_, max_integral_);

    // Derivative term
    double d_term = 0.0;
    if (!first_run_) {
        double derivative = (error - prev_error_) / dt;
        d_term = kd_ * derivative;
    } else {
        first_run_ = false;
    }

    prev_error_ = error;

    double output = p_term + i_term + d_term;
    return clampVal(output, min_output_, max_output_);
}

void PIDController::reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
    first_run_ = true;
}

} // namespace human_call_robot
