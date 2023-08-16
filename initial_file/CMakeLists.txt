cmake_minimum_required(VERSION 2.8.3)
project(initial_file)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(OpenCV REQUIRED)

## Specify additional locations of header files
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
)

## CREATE EXECUTABLE LIBRARY
add_executable(main
        src/clipper.cpp
        src/models.cpp
        src/clipper_extensions.cpp
        src/dubins.cpp
        src/utils.cpp
        src/main.cpp
)


target_link_libraries(main
        ${OpenCV_LIBRARIES}
)
