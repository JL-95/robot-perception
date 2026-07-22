#pragma once

#include "robobites\perception\frame.hpp"

#include <cstdint>
#include <vector>

namespace robobites::perception {

struct GridCell {
    std::int32_t x;
    std::int32_t y;
    float confidence;
};

struct RobotPose {
    float x_m;
    float y_m;
    float heading_rad;
};

std::vector<GridCell> project_lidar(const std::vector<LidarPoint>& points,
                                    const RobotPose& pose,
                                    float cell_size_m);

float average_intensity(const std::vector<LidarPoint>& points);

}
