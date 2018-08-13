# This CMakeLists.txt file must exist at the base of the
# thunderbots package, as required by ROS and catkin
#
# The CMakeLists.txt to actually build our software is
# called from here.

# We need cmake version >= 3.8.2 so that we can set the
# CMAKE_CXX_STANDARD to 17
cmake_minimum_required(VERSION 3.8.2)
project(thunderbots_all)

add_subdirectory(software)
