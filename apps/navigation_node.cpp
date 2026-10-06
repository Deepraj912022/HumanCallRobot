#include <iostream>
#include "navigation/AStarPlanner.hpp"
#include "mapping/OccupancyGrid.hpp"

int main() {
    std::cout << "Starting HumanCallRobot Navigation Node..." << std::endl;
    human_call_robot::OccupancyGrid grid(50, 50, 0.1, -2.5, -2.5);
    human_call_robot::AStarPlanner planner;

    human_call_robot::Pose2D start(0.0, 0.0, 0.0);
    human_call_robot::Pose2D goal(2.0, 2.0, 0.0);
    std::vector<human_call_robot::Pose2D> path;

    if (planner.planPath(grid, start, goal, path)) {
        std::cout << "Navigation Node: Path successfully generated with " << path.size() << " waypoints." << std::endl;
    } else {
        std::cout << "Navigation Node: Failed to find path." << std::endl;
    }
    return 0;
}
