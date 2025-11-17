cmake_minimum_required(VERSION 3.8)
project(waypoint_generator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(waypoint_generator src/waypoint_generator.cpp)
ament_target_dependencies(waypoint_generator rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS waypoint_generator
  DESTINATION lib/${PROJECT_NAME})

ament_package()
