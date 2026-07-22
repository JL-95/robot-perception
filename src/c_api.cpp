#include "robobites\perception\c_api.h"
#include "robobites\perception\frame.hpp"

#include <exception>
#include <string>

namespace {

thread_local std::string last_error;

}

extern "C" {

int rb_decode_frame(const uint8_t* data, size_t length, rb_frame_info* out_info) {
    if (out_info == nullptr) {
        last_error = "result pointer is null";
        return -1;
    }

    try {
        robobites::perception::CameraFrame frame =
            robobites::perception::decode_frame(data, length);

        out_info->width = frame.header.width;
        out_info->height = frame.header.height;
        out_info->channels = frame.channels;
        out_info->lidar_points = frame.header.lidar_points;
        out_info->timestamp_ms = frame.header.timestamp_ms;
        last_error.clear();
        return 0;
    } catch (const std::exception& ex) {
        last_error = ex.what();
        return -1;
    }
}

const char* rb_last_error(void) {
    return last_error.c_str();
}

}
