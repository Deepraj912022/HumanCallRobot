#include "mapping/OccupancyGrid.hpp"

namespace human_call_robot {

OccupancyGrid::OccupancyGrid(int width, int height, double resolution, double origin_x, double origin_y)
    : width_(width), height_(height), resolution_(resolution), origin_(origin_x, origin_y, 0.0) {
    data_.resize(width_ * height_, 0);
}

void OccupancyGrid::setDimensions(int width, int height, double resolution, double origin_x, double origin_y) {
    width_ = width;
    height_ = height;
    resolution_ = resolution;
    origin_ = Pose2D(origin_x, origin_y, 0.0);
    data_.assign(width_ * height_, 0);
}

int8_t OccupancyGrid::getValue(int gx, int gy) const {
    if (isOutOfBounds(gx, gy)) return -1;
    return data_[gy * width_ + gx];
}

void OccupancyGrid::setValue(int gx, int gy, int8_t value) {
    if (!isOutOfBounds(gx, gy)) {
        data_[gy * width_ + gx] = value;
    }
}

bool OccupancyGrid::worldToGrid(double wx, double wy, int& gx, int& gy) const {
    gx = static_cast<int>((wx - origin_.x) / resolution_);
    gy = static_cast<int>((wy - origin_.y) / resolution_);
    return !isOutOfBounds(gx, gy);
}

void OccupancyGrid::gridToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = origin_.x + (gx + 0.5) * resolution_;
    wy = origin_.y + (gy + 0.5) * resolution_;
}

bool OccupancyGrid::isOccupied(int gx, int gy) const {
    int8_t val = getValue(gx, gy);
    return val >= 50;
}

bool OccupancyGrid::isFree(int gx, int gy) const {
    int8_t val = getValue(gx, gy);
    return val >= 0 && val < 50;
}

bool OccupancyGrid::isOutOfBounds(int gx, int gy) const {
    return gx < 0 || gx >= width_ || gy < 0 || gy >= height_;
}

void OccupancyGrid::clear(int8_t default_value) {
    std::fill(data_.begin(), data_.end(), default_value);
}

} // namespace human_call_robot
