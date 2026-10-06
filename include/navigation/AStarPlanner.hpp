#ifndef HUMAN_CALL_ROBOT_NAVIGATION_ASTAR_PLANNER_HPP_
#define HUMAN_CALL_ROBOT_NAVIGATION_ASTAR_PLANNER_HPP_

#include "mapping/OccupancyGrid.hpp"
#include "core/Pose2D.hpp"
#include <vector>
#include <functional>

namespace human_call_robot {

struct Node2D {
    int x;
    int y;
    double g_cost{1e9};
    double h_cost{0.0};
    int parent_x{-1};
    int parent_y{-1};

    double f_cost() const { return g_cost + h_cost; }

    bool operator>(const Node2D& other) const {
        return f_cost() > other.f_cost();
    }
};

class AStarPlanner {
public:
    AStarPlanner();

    bool planPath(const OccupancyGrid& grid, const Pose2D& start, const Pose2D& goal, std::vector<Pose2D>& out_path);

private:
    double calculateHeuristic(int x1, int y1, int x2, int y2) const;
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_NAVIGATION_ASTAR_PLANNER_HPP_