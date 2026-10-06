#ifndef HUMAN_CALL_ROBOT_CONTROL_PID_CONTROLLER_HPP_
#define HUMAN_CALL_ROBOT_CONTROL_PID_CONTROLLER_HPP_

namespace human_call_robot {

class PIDController {
public:
    PIDController();
    PIDController(double kp, double ki, double kd, double min_out = -1.0, double max_out = 1.0);

    void setGains(double kp, double ki, double kd);
    void setOutputLimits(double min_out, double max_out);
    void setIntegralLimits(double min_int, double max_int);

    double compute(double setpoint, double measurement, double dt);
    void reset();

private:
    double kp_{1.0};
    double ki_{0.0};
    double kd_{0.0};

    double min_output_{-1.0};
    double max_output_{1.0};
    double min_integral_{-1.0};
    double max_integral_{1.0};

    double prev_error_{0.0};
    double integral_{0.0};
    bool first_run_{true};
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CONTROL_PID_CONTROLLER_HPP_
