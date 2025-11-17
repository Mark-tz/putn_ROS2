cmake_minimum_required(VERSION 3.8)
project(gpr)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_executable(gpr_path src/gpr_path.cpp)
ament_target_dependencies(gpr_path rclcpp std_msgs)

install(TARGETS gpr_path
  DESTINATION lib/${PROJECT_NAME})

ament_package()





