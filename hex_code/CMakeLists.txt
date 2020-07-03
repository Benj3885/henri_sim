cmake_minimum_required(VERSION 2.8.3)
project(hex_code)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  log_hex
  grid_map_core
  sensor_msgs
  cv_bridge
  tf2
  grid_map_ros
  geometry_msgs
)
find_package(OpenCV REQUIRED core highgui videoio)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(control 
src/main.cpp src/map_nav.cpp src/control.cpp
include/map_nav.h include/control.h
)

target_link_libraries(control 
pthread m ${catkin_LIBRARIES}
${OpenCV_LIBRARIES}
)
