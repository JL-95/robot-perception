#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace caldova::perception {

struct LidarPoint {
    float x;
    float y;
    float intensity;
};

struct FrameHeader {
    std::uint16_t version;
    std::uint16_t header_bytes;
    std::uint32_t width;
    std::uint32_t height;
    std::uint64_t timestamp_ms;
    std::uint32_t camera_bytes;
    std::uint32_t lidar_points;
};

struct CameraFrame {
    FrameHeader header;
    std::uint8_t channels;
    std::vector<std::uint8_t> pixels;
    std::vector<LidarPoint> lidar;
};

class DecodeError : public std::runtime_error {
public:
    explicit DecodeError(const std::string& message) : std::runtime_error(message) {}
};

CameraFrame decode_frame(const std::uint8_t* data, std::size_t length);

} 
