add_library(header STATIC ParametricSpline.cpp)

target_include_directories(header
INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR})