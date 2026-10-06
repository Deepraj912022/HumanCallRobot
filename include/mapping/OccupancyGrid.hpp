#ifndef HUMAN_CALL_ROBOT_MAPPING_OCCUPANCY_GRID_HPP_
#define HUMAN_CALL_ROBOT_MAPPING_OCCUPANCY_GRID_HPP_

#include <vector>
#include <cstdint>
#include "core/Pose2D.hpp"

namespace human_call_robot {

class OccupancyGrid {
public:
    OccupancyGrid(int width = 100, int height = 100, double resolution = 0.05, double origin_x = -2.5, double origin_y = -2.5);

    void setDimensions(int width, int height, double resolution, double origin_x, double origin_y);

    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    Pose2D getOrigin() const { return origin_; }

    int8_t getValue(int gx, int gy) const;
    void setValue(int gx, int gy, int8_t value);

    bool worldToGrid(double wx, double wy, int& gx, int& gy) const;
    void gridToWorld(int gx, int gy, double& wx, double& wy) const;

    bool isOccupied(int gx, int gy) const;
    bool isFree(int gx, int gy) const;
    bool isOutOfBounds(int gx, int gy) const;

    const std::vector<int8_t>& getData() const { return data_; }
    void clear(int8_t default_value = 0);

private:
    int width_{100};
    int height_{100};
    double resolution_{0.05}; // meters per cell
    Pose2D origin_;
    std::vector<int8_t> data_; // 0 = free, 100 = occupied, -1 = unknown
};

} // namespace human_call_robot

#endif // HUMAN_CALL_ROBOT_MAPPING_OCCUPANCY_GRID_HPP_
