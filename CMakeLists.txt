cmake_minimum_required(VERSION 2.8.3)
project(stewart)

SET(Eigen3_INCLUDE_DIRS "/usr/local/include/eigen3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_modules
)

find_package(Eigen3 REQUIRED)

catkin_package(DEPENDS Eigen3)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable(ps4_controller src/ps4_controller.cpp)
target_link_libraries(ps4_controller ${catkin_LIBRARIES})

add_executable(ik src/ik.cpp)
target_link_libraries(ik ${catkin_LIBRARIES})