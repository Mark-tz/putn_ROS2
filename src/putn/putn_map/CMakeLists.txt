cmake_minimum_required(VERSION 3.8)
project(putn_map)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds map1 map2
  DESTINATION share/${PROJECT_NAME})

ament_package()
