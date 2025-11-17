cmake_minimum_required(VERSION 3.8)
project(scout_gazebo_sim)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library(scout_gazebo STATIC src/scout_skid_steer.cpp)
ament_target_dependencies(scout_gazebo rclcpp geometry_msgs std_msgs)

add_executable(scout_skid_steer_controller src/scout_skid_steer_controller.cpp)
ament_target_dependencies(scout_skid_steer_controller rclcpp geometry_msgs std_msgs)
target_link_libraries(scout_skid_steer_controller scout_gazebo)

install(TARGETS scout_gazebo scout_skid_steer_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch worlds
  DESTINATION share/${PROJECT_NAME})

ament_package()
