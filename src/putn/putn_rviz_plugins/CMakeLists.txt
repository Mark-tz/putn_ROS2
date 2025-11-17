cmake_minimum_required(VERSION 3.8)
project(rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)


###########
## Build ##
###########

#find_package(Armadillo REQUIRED)
#include_directories(${ARMADILLO_INCLUDE_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
#find_package(Qt5 COMPONENTS QtCore QtGui REQUIRED)


## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt5_wrap_cpp(MOC_FILES
  src/goal_tool.h
#  src/probmap_display.h
#  src/aerialmap_display.h    
#  src/multi_probmap_display.h      
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/pose_tool.cpp
  src/goal_tool.cpp
#  src/probmap_display.cpp
#  src/aerialmap_display.cpp   
#  src/multi_probmap_display.cpp  
  ${MOC_FILES}
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME} rclcpp rviz_common rviz_rendering pluginlib geometry_msgs)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

# optional assets; only install if present
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/media)
  install(DIRECTORY media/
    DESTINATION share/${PROJECT_NAME}/media)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/icons)
  install(DIRECTORY icons/
    DESTINATION share/${PROJECT_NAME}/icons)
endif()

ament_package()
