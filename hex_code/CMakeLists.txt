cmake_minimum_required(VERSION 2.8.3)
project(hex_code)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src
)

add_executable(control 
src/main.cpp src/laser.cpp src/comm.cpp src/control.cpp src/motion.cpp
include/laser.h include/comm.h include/control.h include/motion.h
)

target_link_libraries(control 
pthread m ${catkin_LIBRARIES}
)
