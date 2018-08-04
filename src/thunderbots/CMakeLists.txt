# This CMakeLists.txt file must exist at the base of the
# thunderbots package, as required by ROS and catkin
#
# The CMakeLists.txt to actually build our software is
# called from here.

cmake_minimum_required(VERSION 3.1)
project(thunderbots)

add_subdirectory(software)
