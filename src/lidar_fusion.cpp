#include "caldova\perception\lidar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace caldova::perception {
namespace {

GridCell to_cell(const LidarPoint& point, const RobotPose& pose, float cell_size_m) {
    const float cos_h = std::cos(pose.heading_rad);
    const float sin_h = std::sin(pose.heading_rad);
    const float world_x = pose.x_m + point.x * cos_h - point.y * sin_h;
    const float world_y = pose.y_m + point.x * sin_h + point.y * cos_h;

    return GridCell{
        static_cast<std::int32_t>(std::floor(world_x / cell_size_m)),
        static_cast<std::int32_t>(std::floor(world_y / cell_size_m)),
        std::max(0.0f, std::min(1.0f, point.intensity))
    };
}

}

std::vector<GridCell> project_lidar(const std::vector<LidarPoint>& points,
                                    const RobotPose& pose,
                                    float cell_size_m) {
    if (cell_size_m <= std::numeric_limits<float>::epsilon()) {
        throw DecodeError("cell size must be positive");
    }

    std::vector<GridCell> cells;
    cells.reserve(points.size());

    for (const LidarPoint& point : points) {
        if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.intensity)) {
            cells.push_back(to_cell(point, pose, cell_size_m));
        }
    }

    return cells;
}

float average_intensity(const std::vector<LidarPoint>& points) {
    if (points.empty()) {
        return 0.0f;
    }

    double sum = 0.0;
    std::size_t count = 0;
    for (const LidarPoint& point : points) {
        if (std::isfinite(point.intensity)) {
            sum += point.intensity;
            ++count;
        }
    }

    return count == 0 ? 0.0f : static_cast<float>(sum / count);
}

}
