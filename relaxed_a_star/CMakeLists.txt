cmake_minimum_required(VERSION 2.8.3)
project(relaxed_a_star)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  costmap_2d
  nav_core
)

# # System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)


# find_package(OpenCV 2 REQUIRED)

# # Uncomment this if the package has a setup.py. This macro ensures
# # modules and global scripts declared therein get installed
# # See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

# ##################################
# # catkin specific configuration ##
# ##################################
# # The catkin_package macro generates cmake config files for your package
# # Declare things to be passed to dependent projects
# # INCLUDE_DIRS: uncomment this if your package contains header files
# # LIBRARIES: libraries you create in this project that dependent projects also need
# # CATKIN_DEPENDS: catkin_packages dependent projects also need
# # DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    relaxed_a_star
  # CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    nav_core
  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

set(RELAXED_A_STAR_DATA_SOURCES  
  include/relaxed_a_star/ras_param_manager.h
	src/ras_param_manager.cpp

  include/relaxed_a_star/path_planner_types.h
  )
add_library(ras_data_classes ${RELAXED_A_STAR_DATA_SOURCES})


set(RELAXED_A_STAR_SOURCES
	include/relaxed_a_star/relaxed_a_star.h
	src/relaxed_a_star.cpp
  )
add_library(relaxed_a_star ${RELAXED_A_STAR_SOURCES})

set_target_properties(ras_data_classes PROPERTIES LINKER_LANGUAGE CXX)

target_link_libraries(relaxed_a_star 
  ras_data_classes
  ${catkin_LIBRARIES})