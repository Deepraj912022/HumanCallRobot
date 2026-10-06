#include "mapping/Costmap.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot::mapping {

bool Costmap::initialize()
{
    initialized_ = true;
    clear();

    return true;
}

void Costmap::reset()
{
    clear();
}

void Costmap::shutdown()
{
    clear();
    initialized_ = false;
}

bool Costmap::isInitialized() const
{
    return initialized_;
}

bool Costmap::isValid() const
{
    return (
        initialized_ &&
        width_ > 0 &&
        height_ > 0 &&
        resolution_ > 0.0 &&
        std::isfinite(resolution_) &&
        std::isfinite(originX_) &&
        std::isfinite(originY_) &&
        data_.size() ==
            static_cast<std::size_t>(width_) *
            static_cast<std::size_t>(height_)
    );
}

bool Costmap::setGeometry(
    int width,
    int height,
    double resolution,
    double originX,
    double originY)
{
    if (!initialized_)
    {
        return false;
    }

    if (width <= 0 ||
        height <= 0 ||
        !std::isfinite(resolution) ||
        resolution <= 0.0 ||
        !std::isfinite(originX) ||
        !std::isfinite(originY))
    {
        return false;
    }

    width_ = width;
    height_ = height;
    resolution_ = resolution;
    originX_ = originX;
    originY_ = originY;

    data_.assign(
        static_cast<std::size_t>(width_) *
        static_cast<std::size_t>(height_),
        FREE_SPACE
    );

    return true;
}

int Costmap::width() const
{
    return width_;
}

int Costmap::height() const
{
    return height_;
}

double Costmap::resolution() const
{
    return resolution_;
}

double Costmap::originX() const
{
    return originX_;
}

double Costmap::originY() const
{
    return originY_;
}

bool Costmap::inBounds(
    int gridX,
    int gridY) const
{
    return (
        gridX >= 0 &&
        gridX < width_ &&
        gridY >= 0 &&
        gridY < height_
    );
}

bool Costmap::worldToGrid(
    double worldX,
    double worldY,
    int& gridX,
    int& gridY) const
{
    if (!isValid())
    {
        return false;
    }

    if (!std::isfinite(worldX) ||
        !std::isfinite(worldY))
    {
        return false;
    }

    const double gx =
        (worldX - originX_) / resolution_;

    const double gy =
        (worldY - originY_) / resolution_;

    if (!std::isfinite(gx) ||
        !std::isfinite(gy))
    {
        return false;
    }

    gridX = static_cast<int>(std::floor(gx));
    gridY = static_cast<int>(std::floor(gy));

    return inBounds(gridX, gridY);
}

bool Costmap::gridToWorld(
    int gridX,
    int gridY,
    double& worldX,
    double& worldY) const
{
    if (!isValid() ||
        !inBounds(gridX, gridY))
    {
        return false;
    }

    worldX =
        originX_ +
        (static_cast<double>(gridX) + 0.5) *
        resolution_;

    worldY =
        originY_ +
        (static_cast<double>(gridY) + 0.5) *
        resolution_;

    return (
        std::isfinite(worldX) &&
        std::isfinite(worldY)
    );
}

bool Costmap::setCost(
    int gridX,
    int gridY,
    std::uint8_t cost)
{
    if (!inBounds(gridX, gridY))
    {
        return false;
    }

    data_[index(gridX, gridY)] = cost;

    return true;
}

std::uint8_t Costmap::getCost(
    int gridX,
    int gridY) const
{
    if (!inBounds(gridX, gridY))
    {
        return LETHAL_OBSTACLE;
    }

    return data_[index(gridX, gridY)];
}

bool Costmap::isFree(
    int gridX,
    int gridY) const
{
    return (
        inBounds(gridX, gridY) &&
        getCost(gridX, gridY) < INSCRIBED_OBSTACLE
    );
}

bool Costmap::isOccupied(
    int gridX,
    int gridY) const
{
    return (
        !inBounds(gridX, gridY) ||
        getCost(gridX, gridY) >= INSCRIBED_OBSTACLE
    );
}

void Costmap::clear()
{
    if (width_ <= 0 || height_ <= 0)
    {
        data_.clear();
        return;
    }

    data_.assign(
        static_cast<std::size_t>(width_) *
        static_cast<std::size_t>(height_),
        FREE_SPACE
    );
}

void Costmap::setAll(
    std::uint8_t cost)
{
    std::fill(
        data_.begin(),
        data_.end(),
        cost
    );
}

bool Costmap::inflate(
    double radius)
{
    if (!isValid() ||
        !std::isfinite(radius) ||
        radius < 0.0)
    {
        return false;
    }

    if (radius == 0.0)
    {
        return true;
    }

    const int cellRadius =
        static_cast<int>(
            std::ceil(radius / resolution_)
        );

    std::vector<std::uint8_t> original =
        data_;

    for (int y = 0; y < height_; ++y)
    {
        for (int x = 0; x < width_; ++x)
        {
            const std::size_t currentIndex =
                index(x, y);

            if (original[currentIndex] <
                INSCRIBED_OBSTACLE)
            {
                continue;
            }

            for (int dy = -cellRadius;
                 dy <= cellRadius;
                 ++dy)
            {
                for (int dx = -cellRadius;
                     dx <= cellRadius;
                     ++dx)
                {
                    if (dx == 0 && dy == 0)
                    {
                        continue;
                    }

                    const int nx = x + dx;
                    const int ny = y + dy;

                    if (!inBounds(nx, ny))
                    {
                        continue;
                    }

                    const double distance =
                        std::sqrt(
                            static_cast<double>(dx * dx + dy * dy)
                        ) *
                        resolution_;

                    if (distance > radius)
                    {
                        continue;
                    }

                    const double ratio =
                        1.0 -
                        (distance / radius);

                    const auto inflatedCost =
                        static_cast<std::uint8_t>(
                            std::clamp(
                                ratio * 253.0,
                                1.0,
                                253.0
                            )
                        );

                    const std::size_t neighborIndex =
                        index(nx, ny);

                    data_[neighborIndex] =
                        std::max(
                            data_[neighborIndex],
                            inflatedCost
                        );
                }
            }
        }
    }

    return true;
}

const std::vector<std::uint8_t>&
Costmap::data() const
{
    return data_;
}

std::size_t Costmap::index(
    int gridX,
    int gridY) const
{
    return (
        static_cast<std::size_t>(gridY) *
        static_cast<std::size_t>(width_) +
        static_cast<std::size_t>(gridX)
    );
}

} // namespace robot::mapping