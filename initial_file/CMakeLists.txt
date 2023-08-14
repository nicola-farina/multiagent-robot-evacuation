cmake_minimum_required(VERSION 2.8.3)
project(initial_file)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Specify additional locations of header files
include_directories(
        include
)

## CREATE EXECUTABLE LIBRARY
add_executable(main
        src/main.cpp
        src/clipper.cpp
        src/models.cpp
)


# Executable to test the implemented functions
# add_executable(functions-testing src/main.cpp src/dubins.cpp src/utils.cpp src/clipper.cpp src/clipper_addons.cpp src/graph.cpp src/open_edges.cpp src/visgraph.cpp src/graphPrint.cpp)

# target_link_libraries(functions-testing
#   ${OpenCV_LIBRARIES}
# )
