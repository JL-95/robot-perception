#include "robobites\perception\frame.hpp"

#include <cstring>
#include <limits>

namespace robobites::perception {
namespace {

constexpr std::uint32_t kMagic = 0x46504252;
constexpr std::uint16_t kVersion = 1;
constexpr std::size_t kHeaderBytes = 32;
constexpr std::uint8_t kChannels = 3;

std::uint16_t read_u16(const std::uint8_t* p) {
    return static_cast<std::uint16_t>(p[0] | (p[1] << 8));
}

std::uint32_t read_u32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) |
           (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) |
           (static_cast<std::uint32_t>(p[3]) << 24);
}

std::uint64_t read_u64(const std::uint8_t* p) {
    return static_cast<std::uint64_t>(read_u32(p)) |
           (static_cast<std::uint64_t>(read_u32(p + 4)) << 32);
}

FrameHeader parse_header(const std::uint8_t* data, std::size_t length) {
    if (length < kHeaderBytes) {
        throw DecodeError("frame is shorter than the Robobites perception header");
    }

    if (read_u32(data) != kMagic) {
        throw DecodeError("frame magic does not match RBPF");
    }

    FrameHeader header{};
    header.version = read_u16(data + 4);
    header.header_bytes = read_u16(data + 6);
    header.width = read_u32(data + 8);
    header.height = read_u32(data + 12);
    header.timestamp_ms = read_u64(data + 16);
    header.camera_bytes = read_u32(data + 24);
    header.lidar_points = read_u32(data + 28);

    if (header.version != kVersion || header.header_bytes < kHeaderBytes) {
        throw DecodeError("unsupported perception frame version");
    }

    return header;
}

std::vector<std::uint8_t> decode_camera(const FrameHeader& header,
                                        const std::uint8_t* payload,
                                        std::size_t payload_available) {
    const std::uint32_t pixel_count = header.width * header.height;
    const std::uint32_t storage_bytes = pixel_count * kChannels;

    if (payload_available < header.camera_bytes) {
        throw DecodeError("camera payload ended early");
    }

    std::vector<std::uint8_t> pixels(storage_bytes);
    std::memcpy(pixels.data(), payload, header.camera_bytes);
    return pixels;
}

std::vector<LidarPoint> decode_lidar(const FrameHeader& header,
                                     const std::uint8_t* payload,
                                     std::size_t payload_available) {
    const std::size_t bytes = static_cast<std::size_t>(header.lidar_points) * sizeof(LidarPoint);
    if (payload_available < bytes) {
        throw DecodeError("LiDAR payload ended early");
    }

    std::vector<LidarPoint> points(header.lidar_points);
    std::memcpy(points.data(), payload, bytes);
    return points;
}

}

CameraFrame decode_frame(const std::uint8_t* data, std::size_t length) {
    if (data == nullptr) {
        throw DecodeError("frame buffer is null");
    }

    FrameHeader header = parse_header(data, length);
    if (header.header_bytes > length) {
        throw DecodeError("frame header extends past buffer");
    }

    const std::uint8_t* cursor = data + header.header_bytes;
    std::size_t remaining = length - header.header_bytes;

    std::vector<std::uint8_t> pixels = decode_camera(header, cursor, remaining);
    cursor += header.camera_bytes;
    remaining -= header.camera_bytes;

    std::vector<LidarPoint> points = decode_lidar(header, cursor, remaining);

    CameraFrame frame{};
    frame.header = header;
    frame.channels = kChannels;
    frame.pixels = std::move(pixels);
    frame.lidar = std::move(points);
    return frame;
}

}
