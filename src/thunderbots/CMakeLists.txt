# This CMakeLists.txt file must exist at the base of the
# thunderbots package, as required by ROS and catkin
#
# The CMakeLists.txt to actually build our software is
# called from here.

# We need cmake version >= 3.8.2 so that we can set the
# CMAKE_CXX_STANDARD to 17
cmake_minimum_required(VERSION 3.8.2)
project(thunderbots_all)

# All the test files in the shared code directory
# This is put at top level since shared files may be used across software and firmware,
# and globbing the files from here removes the need for relative paths
# (i.e. paths with .. in them)
file(GLOB_RECURSE SHARED_UTIL_TEST LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/test/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/test/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/test/*.h
        )

# All the source files in the shared code directory. Initially this includes the test
# files because of the globbing
file(GLOB_RECURSE SHARED_UTIL_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/shared/*.h
        )

# Remove the test files from the SHARED_UTIL_SRC so that we are only left with source
# files, and no test files
list(REMOVE_ITEM SHARED_UTIL_SRC ${SHARED_UTIL_TEST})

add_subdirectory(software)
add_subdirectory(firmware)
