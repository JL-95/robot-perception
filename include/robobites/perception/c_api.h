#pragma once

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32) && defined(RB_PERCEPTION_BUILD)
#define RB_API __declspec(dllexport)
#elif defined(_WIN32)
#define RB_API __declspec(dllimport)
#else
#define RB_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rb_frame_info {
    uint32_t width;
    uint32_t height;
    uint8_t channels;
    uint32_t lidar_points;
    uint64_t timestamp_ms;
} rb_frame_info;

RB_API int rb_decode_frame(const uint8_t* data, size_t length, rb_frame_info* out_info);
RB_API const char* rb_last_error(void);

#ifdef __cplusplus
}
#endif
