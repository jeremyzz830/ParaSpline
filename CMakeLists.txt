cmake_minimum_required(VERSION 3.10)

project(ParaSpline VERSION 1.0)

message(STATUS "${PROJECT_SOURCE_DIR}/lib")
message(STATUS "${EIGEN3_INCLUDE_DIR}")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeConfig")
include(building_output)

find_package(Eigen3 REQUIRED)

# include_directories(EIGEN3_INCLUDE_DIR)

add_subdirectory(lib)

add_subdirectory(src)