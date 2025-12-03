cmake_minimum_required(VERSION 3.8)
project(local_planner)

find_package(ament_cmake REQUIRED)

install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/local_planner.py
  scripts/controller.py
  scripts/pcd_publisher.py
  scripts/simple_sim.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  scripts/MPC.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()
