cmake_minimum_required(VERSION 2.8.3)
project(log_hex)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  message_generation
)

add_message_files(
  FILES
  log.msg
)

generate_messages(DEPENDENCIES geometry_msgs sensor_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src
)

add_executable(log 
src/log.cpp
)

target_link_libraries(log ${catkin_LIBRARIES})
add_dependencies(log ${catkin_EXPORTED_TARGETS})