cmake_minimum_required(VERSION 3.8)
project(putn)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(global_planning_node
    src/global_planning_node.cpp
    src/PUTN_planner.cpp
    src/PUTN_astar_planner.cpp
    src/PUTN_classes.cpp
    src/PUTN_vis.cpp)
ament_target_dependencies(global_planning_node rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs std_msgs pcl_conversions)
target_link_libraries(global_planning_node ${PCL_LIBRARIES})

add_executable(local_obs_node
    src/local_obs.cpp
    src/PUTN_planner.cpp
    src/PUTN_classes.cpp
    src/PUTN_vis.cpp)
ament_target_dependencies(local_obs_node rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs nav_msgs sensor_msgs visualization_msgs std_msgs pcl_conversions)
target_link_libraries(local_obs_node ${PCL_LIBRARIES})

install(TARGETS global_planning_node local_obs_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()




