#include "control/DifferentialDrive.hpp"

namespace human_call_robot {

DifferentialDrive::DifferentialDrive(double wheel_radius, double track_width)
    : wheel_radius_(wheel_radius), track_width_(track_width) {}

void DifferentialDrive::setGeometry(double wheel_radius, double track_width) {
    wheel_radius_ = wheel_radius;
    track_width_ = track_width;
}

Twist DifferentialDrive::forwardKinematics(double left_wheel_vel, double right_wheel_vel) const {
    double v = (right_wheel_vel + left_wheel_vel) / 2.0;
    double w = (right_wheel_vel - left_wheel_vel) / track_width_;
    return Twist(v, 0.0, w);
}

WheelVelocities DifferentialDrive::inverseKinematics(const Twist& twist) const {
    WheelVelocities wheels;
    wheels.left = twist.linear_x - (twist.angular_z * track_width_ / 2.0);
    wheels.right = twist.linear_x + (twist.angular_z * track_width_ / 2.0);
    return wheels;
}

} // namespace human_call_robot
