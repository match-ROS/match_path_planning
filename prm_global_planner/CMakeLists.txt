cmake_minimum_required(VERSION 2.8.3)
project(prm_global_planner)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_core
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES prm_global_planner
  CATKIN_DEPENDS nav_core roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(prm_global_planner
    include/prm_global_planner/PRM_ros.h
    src/PRM_ros.cpp
)

