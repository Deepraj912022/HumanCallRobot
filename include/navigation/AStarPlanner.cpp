#include "navigation/AStarPlanner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace robot::navigation {

bool AStarPlanner::initialize()
{
    initialized_ = true;
    lastPath_ = GlobalPath{};
    return true;
}

GlobalPath AStarPlanner::plan(
    const robot::core::Pose2D& start,
    const robot::core::Pose2D& goal,
    const robot::mapping::Costmap& costmap)
{
    GlobalPath result;

    if (!initialized_)
    {
        return result;
    }

    if (!std::isfinite(start.x) ||
        !std::isfinite(start.y) ||
        !std::isfinite(start.theta) ||
        !std::isfinite(goal.x) ||
        !std::isfinite(goal.y) ||
        !std::isfinite(goal.theta))
    {
        return result;
    }

    GridNode startNode;
    GridNode goalNode;

    if (!worldToGrid(start, costmap, startNode))
    {
        return result;
    }

    if (!worldToGrid(goal, costmap, goalNode))
    {
        return result;
    }

    if (!isTraversable(startNode, costmap) ||
        !isTraversable(goalNode, costmap))
    {
        return result;
    }

    if (startNode == goalNode)
    {
        PathPoint point;

        point.pose = start;
        point.distanceFromStart = 0.0;
        point.cost = 0.0;

        result.points.push_back(point);
        result.totalCost = 0.0;

        lastPath_ = result;

        return result;
    }

    std::priority_queue<
        OpenNode,
        std::vector<OpenNode>,
        OpenNodeCompare
    > openSet;

    std::unordered_map<
        GridNode,
        double,
        GridNodeHash
    > gScore;

    std::unordered_map<
        GridNode,
        GridNode,
        GridNodeHash
    > cameFrom;

    std::unordered_set<
        GridNode,
        GridNodeHash
    > closedSet;

    gScore[startNode] = 0.0;

    openSet.push(
        OpenNode{
            startNode,
            heuristic(startNode, goalNode)
        }
    );

    while (!openSet.empty())
    {
        const GridNode current = openSet.top().node;
        openSet.pop();

        if (closedSet.find(current) != closedSet.end())
        {
            continue;
        }

        if (current == goalNode)
        {
            result = reconstructPath(
                startNode,
                goalNode,
                cameFrom,
                costmap
            );

            if (result.valid())
            {
                lastPath_ = result;
            }

            return result;
        }

        closedSet.insert(current);

        const auto neighbors = getNeighbors(current);

        for (const auto& neighbor : neighbors)
        {
            if (closedSet.find(neighbor) != closedSet.end())
            {
                continue;
            }

            if (!isTraversable(neighbor, costmap))
            {
                continue;
            }

            const double movementCost =
                distance(current, neighbor);

            const double tentativeG =
                gScore[current] + movementCost;

            const auto existingScore =
                gScore.find(neighbor);

            if (existingScore == gScore.end() ||
                tentativeG < existingScore->second)
            {
                cameFrom[neighbor] = current;
                gScore[neighbor] = tentativeG;

                const double fScore =
                    tentativeG +
                    heuristic(neighbor, goalNode);

                openSet.push(
                    OpenNode{
                        neighbor,
                        fScore
                    }
                );
            }
        }
    }

    // No path found.
    lastPath_ = GlobalPath{};
    return result;
}

bool AStarPlanner::isPathValid(
    const GlobalPath& path,
    const robot::mapping::Costmap& costmap) const
{
    if (!initialized_)
    {
        return false;
    }

    if (!path.valid())
    {
        return false;
    }

    for (const auto& point : path.points)
    {
        GridNode node;

        if (!worldToGrid(
                point.pose,
                costmap,
                node))
        {
            return false;
        }

        if (!isTraversable(node, costmap))
        {
            return false;
        }
    }

    return true;
}

GlobalPath AStarPlanner::getLastPath() const
{
    return lastPath_;
}

bool AStarPlanner::isInitialized() const
{
    return initialized_;
}

void AStarPlanner::reset()
{
    lastPath_ = GlobalPath{};
}

void AStarPlanner::shutdown()
{
    initialized_ = false;
    lastPath_ = GlobalPath{};
}

double AStarPlanner::heuristic(
    const GridNode& a,
    const GridNode& b)
{
    const double dx =
        static_cast<double>(a.x - b.x);

    const double dy =
        static_cast<double>(a.y - b.y);

    // Euclidean distance.
    return std::sqrt(
        dx * dx +
        dy * dy
    );
}

double AStarPlanner::distance(
    const GridNode& a,
    const GridNode& b)
{
    const double dx =
        static_cast<double>(a.x - b.x);

    const double dy =
        static_cast<double>(a.y - b.y);

    return std::sqrt(
        dx * dx +
        dy * dy
    );
}

std::vector<AStarPlanner::GridNode>
AStarPlanner::getNeighbors(
    const GridNode& node)
{
    return {
        {node.x + 1, node.y},
        {node.x - 1, node.y},
        {node.x, node.y + 1},
        {node.x, node.y - 1},

        {node.x + 1, node.y + 1},
        {node.x - 1, node.y + 1},
        {node.x + 1, node.y - 1},
        {node.x - 1, node.y - 1}
    };
}

bool AStarPlanner::worldToGrid(
    const robot::core::Pose2D& pose,
    const robot::mapping::Costmap& costmap,
    GridNode& node) const
{
    /*
     * This conversion depends on the concrete Costmap API.
     *
     * The current Costmap interface must expose:
     *
     *     worldToGrid(x, y, gridX, gridY)
     *
     * before this implementation can be compiled.
     */

    (void)pose;
    (void)costmap;
    (void)node;

    return false;
}

robot::core::Pose2D AStarPlanner::gridToWorld(
    const GridNode& node,
    const robot::mapping::Costmap& costmap) const
{
    /*
     * This conversion depends on the concrete Costmap API.
     *
     * The Costmap implementation will eventually provide:
     *
     *     gridToWorld(gridX, gridY)
     */

    (void)node;
    (void)costmap;

    return robot::core::Pose2D{};
}

bool AStarPlanner::isTraversable(
    const GridNode& node,
    const robot::mapping::Costmap& costmap) const
{
    /*
     * This depends on the concrete Costmap API.
     *
     * Required operations:
     *
     *     - bounds checking
     *     - occupancy/cost lookup
     */

    (void)node;
    (void)costmap;

    return false;
}

GlobalPath AStarPlanner::reconstructPath(
    const GridNode& start,
    const GridNode& goal,
    const std::unordered_map<
        GridNode,
        GridNode,
        GridNodeHash
    >& cameFrom,
    const robot::mapping::Costmap& costmap) const
{
    GlobalPath path;

    std::vector<GridNode> nodes;

    GridNode current = goal;

    nodes.push_back(current);

    while (!(current == start))
    {
        const auto iterator =
            cameFrom.find(current);

        if (iterator == cameFrom.end())
        {
            return GlobalPath{};
        }

        current = iterator->second;
        nodes.push_back(current);
    }

    std::reverse(
        nodes.begin(),
        nodes.end()
    );

    double accumulatedDistance = 0.0;

    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        PathPoint point;

        point.pose =
            gridToWorld(
                nodes[i],
                costmap
            );

        if (i > 0)
        {
            accumulatedDistance +=
                distance(
                    nodes[i - 1],
                    nodes[i]
                );
        }

        point.distanceFromStart =
            accumulatedDistance;

        point.cost =
            accumulatedDistance;

        if (!point.valid())
        {
            return GlobalPath{};
        }

        path.points.push_back(point);
    }

    path.totalCost =
        accumulatedDistance;

    return path;
}

} // namespace robot::navigation