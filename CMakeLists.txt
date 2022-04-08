cmake_minimum_required(VERSION 3.0.2)
project(geranos_pole_trajectory)

find_package(catkin_simple REQUIRED)
find_package(doxygen_catkin REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
add_definitions(-std=c++11 -fPIC -Werror -Wall -Wno-reorder -Wno-unknown-pragmas)

#############
# LIBRARIES #
#############

cs_add_library(geranos_pole_trajectory_lib
  src/pole_trajectory_node.cpp
)

cs_add_executable(pole_trajectory_node
  src/pole_trajectory_node.cpp
)

include_directories(SYSTEM "${LIB_DIR}/Include")


##########
# EXPORT #
##########
cs_install()
cs_export()