cmake_minimum_required(VERSION 3.8)
project(evacuation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Add the following line at the beginning of the CMakeLists.txt
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(obstacles_msgs REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
        include
)

add_executable(evacuation
        src/evacuation.cpp
)

ament_target_dependencies(evacuation
  rclcpp
  geometry_msgs
  obstacles_msgs
  nav_msgs
  nav2_msgs
  tf2
  tf2_ros
  rclcpp_action
  rclcpp_components
  std_srvs
)


# Install the executable
install(TARGETS evacuation
  DESTINATION lib/${PROJECT_NAME}
)

# Add the install() command after ament_package()
install(DIRECTORY
        launch
  DESTINATION share/${PROJECT_NAME})

# ament_package() should be called before install
ament_package()
