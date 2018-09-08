# This CMakeLists.txt file must exist at the base of the
# thunderbots package, as required by ROS and catkin
#
# The CMakeLists.txt to actually build our software is
# called from here.

# We need cmake version >= 3.8.2 so that we can set the
# CMAKE_CXX_STANDARD to 17
cmake_minimum_required(VERSION 3.8.2)
project(thunderbots_all)

# Create a list of files from the shared folder
# This is put at top level since shared files may be used across software and firmware. Removes need for relative paths
# (i.e. paths with .. in them)
file(GLOB_RECURSE SHARED_UTIL_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/*.h
        )

add_subdirectory(software)
