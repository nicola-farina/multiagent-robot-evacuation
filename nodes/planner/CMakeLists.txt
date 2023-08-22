cmake_minimum_required(VERSION 3.8)
project(planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the following line at the beginning of the CMakeLists.txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)
find_package(roadmap_msgs REQUIRED)

include_directories(
        include
)

add_executable(planner
        src/planner.cpp
        src/clipper.cpp
        src/clipper_extensions.cpp
        src/models.cpp
        src/vgraph.cpp
        src/dubins.cpp
        src/utils.cpp
)

ament_target_dependencies(planner
  rclcpp
  geometry_msgs
  obstacles_msgs
  roadmap_msgs
)


# Install the executable
install(TARGETS planner
  DESTINATION lib/${PROJECT_NAME}
)

# Add the install() command after ament_package()
install(DIRECTORY
        launch
  DESTINATION share/${PROJECT_NAME})

# ament_package() should be called before install
ament_package()
