cmake_minimum_required(VERSION 2.8.3)
project(stewart)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ps4_controller src/ps4_controller.cpp)
target_link_libraries(ps4_controller ${catkin_LIBRARIES})