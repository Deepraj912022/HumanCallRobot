#include <iostream>
#include "control/DifferentialDrive.hpp"

int main() {
    std::cout << "Starting HumanCallRobot Hardware Interface Node..." << std::endl;
    human_call_robot::DifferentialDrive drive(0.1, 0.5);
    auto wheels = drive.inverseKinematics(human_call_robot::Twist(0.5, 0.1));
    std::cout << "Target Wheel Speeds - Left: " << wheels.left << " m/s, Right: " << wheels.right << " m/s" << std::endl;
    return 0;
}
