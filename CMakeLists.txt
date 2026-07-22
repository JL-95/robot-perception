cmake_minimum_required(VERSION 3.16)
project(robot_perception VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(robobites_perception SHARED
    src/frame_decoder.cpp
    src/lidar_fusion.cpp
    src/c_api.cpp
)

target_include_directories(robobites_perception
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_compile_definitions(robobites_perception PRIVATE RB_PERCEPTION_BUILD)

if(MSVC)
    target_compile_options(robobites_perception PRIVATE /W4 /permissive-)
else()
    target_compile_options(robobites_perception PRIVATE -Wall -Wextra -pedantic)
endif()
