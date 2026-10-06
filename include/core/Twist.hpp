#ifndef HUMAN_CALL_ROBOT_CORE_TWIST_HPP_
#define HUMAN_CALL_ROBOT_CORE_TWIST_HPP_

namespace human_call_robot {

class Twist {
public:
    double linear_x;  // Linear velocity in m/s
    double linear_y;  // Lateral velocity in m/s (0 for differential drive)
    double angular_z; // Angular velocity in rad/s

    Twist() : linear_x(0.0), linear_y(0.0), angular_z(0.0) {}
    Twist(double vx, double vy, double wz)
        : linear_x(vx), linear_y(vy), angular_z(wz) {}
    Twist(double vx, double wz)
        : linear_x(vx), linear_y(0.0), angular_z(wz) {}
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_CORE_TWIST_HPP_
