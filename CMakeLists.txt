cmake_minimum_required(VERSION 2.8.3)
project(my_sim)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
include
)

cs_add_library(my_sim_vr_pose
src/my_sim_vr_pose.cpp
)

cs_install()
cs_export()
