cmake_minimum_required(VERSION 3.10)
project(open3d_part)

set(CMAKE_CXX_STANDARD 14)
SET (READJSON_LIB_TYPE SHARED)

SET(CMAKE_BUILD_TYPE Release)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})


add_subdirectory(open3d/3dpart/rply)

#include_directories(open3d)

add_subdirectory(open3d)