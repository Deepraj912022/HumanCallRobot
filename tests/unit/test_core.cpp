#include <iostream>
#include <cassert>
#include "core/Vector2D.hpp"
#include "core/Pose2D.hpp"
#include "control/PIDController.hpp"
#include "control/DifferentialDrive.hpp"

using namespace human_call_robot;

void testVector2D() {
    Vector2D v1(3.0, 4.0);
    assert(v1.length() == 5.0);

    Vector2D v2(1.0, 1.0);
    Vector2D sum = v1 + v2;
    assert(sum.x == 4.0 && sum.y == 5.0);
    std::cout << "✔ Vector2D test passed." << std::endl;
}

void testPIDController() {
    PIDController pid(1.0, 0.1, 0.05, -10.0, 10.0);
    double out = pid.compute(10.0, 0.0, 0.1);
    assert(out > 0);
    std::cout << "✔ PIDController test passed." << std::endl;
}

void testDifferentialDrive() {
    DifferentialDrive drive(0.1, 0.5);
    WheelVelocities wheels = drive.inverseKinematics(Twist(1.0, 0.0));
    assert(wheels.left == 1.0 && wheels.right == 1.0);
    std::cout << "✔ DifferentialDrive test passed." << std::endl;
}

int main() {
    std::cout << "Running Core Unit Tests..." << std::endl;
    testVector2D();
    testPIDController();
    testDifferentialDrive();
    std::cout << "✅ All Core Unit Tests Passed Successfully!" << std::endl;
    return 0;
}
