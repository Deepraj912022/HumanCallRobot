#ifndef HUMAN_CALL_ROBOT_CONTROL_DIFFERENTIAL_DRIVE_HPP_
#define HUMAN_CALL_ROBOT_CONTROL_DIFFERENTIAL_DRIVE_HPP_

#include "core/Twist.hpp"
#include <utility>

namespace human_call_robot {

struct WheelVelocities {
    double left{0.0};  // rad/s or m/s
    double right{0.0}; // rad/s or m/s
};

class DifferentialDrive {
public:
    DifferentialDrive(double wheel_radius = 0.1, double track_width = 0.5);

    void setGeometry(double wheel_radius, double track_width);

    // Forward Kinematics: Wheel velocities (m/s) -> Chassis Twist (linear, angular)
    Twist forwardKinematics(double left_wheel_vel, double right_wheel_vel) const;

    // Inverse Kinematics: Chassis Twist -> Wheel velocities (m/s)
    WheelVelocities inverseKinematics(const Twist& twist) const;

private:
    double wheel_radius_{0.1}; // meters
    double track_width_{0.5};  // meters distance between wheels
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CONTROL_DIFFERENTIAL_DRIVE_HPP_
