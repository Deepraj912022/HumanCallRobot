#include "navigation/AStarPlanner.hpp"
#include <queue>
#include <cmath>
#include <algorithm>
#include <map>

namespace human_call_robot {

AStarPlanner::AStarPlanner() {}

double AStarPlanner::calculateHeuristic(int x1, int y1, int x2, int y2) const {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy); // Euclidean distance
}

bool AStarPlanner::planPath(const OccupancyGrid& grid, const Pose2D& start, const Pose2D& goal, std::vector<Pose2D>& out_path) {
    out_path.clear();

    int start_gx, start_gy, goal_gx, goal_gy;
    if (!grid.worldToGrid(start.x, start.y, start_gx, start_gy) ||
        !grid.worldToGrid(goal.x, goal.y, goal_gx, goal_gy)) {
        return false;
    }

    if (grid.isOccupied(goal_gx, goal_gy)) {
        return false;
    }

    int width = grid.getWidth();
    int height = grid.getHeight();

    auto nodeKey = [width](int x, int y) { return y * width + x; };

    std::priority_queue<Node2D, std::vector<Node2D>, std::greater<Node2D>> open_set;
    std::map<int, Node2D> all_nodes;

    Node2D start_node{start_gx, start_gy, 0.0, calculateHeuristic(start_gx, start_gy, goal_gx, goal_gy), -1, -1};
    open_set.push(start_node);
    all_nodes[nodeKey(start_gx, start_gy)] = start_node;

    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const double move_cost[8] = {1.0, 1.0, 1.0, 1.0, 1.414, 1.414, 1.414, 1.414};

    bool found = false;

    while (!open_set.empty()) {
        Node2D current = open_set.top();
        open_set.pop();

        if (current.x == goal_gx && current.y == goal_gy) {
            found = true;
            break;
        }

        for (int i = 0; i < 8; ++i) {
            int nx = current.x + dx[i];
            int ny = current.y + dy[i];

            if (grid.isOutOfBounds(nx, ny) || grid.isOccupied(nx, ny)) {
                continue;
            }

            double new_g = current.g_cost + move_cost[i];
            int n_key = nodeKey(nx, ny);

            if (all_nodes.find(n_key) == all_nodes.end() || new_g < all_nodes[n_key].g_cost) {
                Node2D neighbor{nx, ny, new_g, calculateHeuristic(nx, ny, goal_gx, goal_gy), current.x, current.y};
                all_nodes[n_key] = neighbor;
                open_set.push(neighbor);
            }
        }
    }

    if (!found) return false;

    // Reconstruct path
    int curr_x = goal_gx;
    int curr_y = goal_gy;
    std::vector<Pose2D> reversed_path;

    while (curr_x != -1 && curr_y != -1) {
        double wx, wy;
        grid.gridToWorld(curr_x, curr_y, wx, wy);
        reversed_path.push_back(Pose2D(wx, wy, 0.0));

        int key = nodeKey(curr_x, curr_y);
        if (all_nodes.find(key) == all_nodes.end()) break;
        Node2D n = all_nodes[key];
        curr_x = n.parent_x;
        curr_y = n.parent_y;
    }

    std::reverse(reversed_path.begin(), reversed_path.end());
    out_path = reversed_path;
    return true;
}

} // namespace human_call_robot
