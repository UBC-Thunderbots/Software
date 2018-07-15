cmake_minimum_required(VERSION 3.1)
project(thunderbots)

# Build in "Release" (with lots of compiler optimizations) by default
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

####################
## Compiler Flags ##
####################

# Set cmake standard to c++ 17
add_definitions(-std=c++17)
add_definitions(-Wno-deprecated) # Don't warn about including "old" headers
add_definitions(-fno-common) # Do not allow multiple definitions of the same global variable

#############################################
## Find and include pacakges and libraries ##
#############################################

# Call our CMake file to include and build protobuf
include("${CMAKE_CURRENT_SOURCE_DIR}/proto/build_proto.cmake")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        thunderbots_msgs
        )

catkin_package(
        INCLUDE_DIRS
        LIBRARIES thunderbots
        CATKIN_DEPENDS roscpp std_msgs
        DEPENDS message_runtime
)

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROTOBUF_INCLUDE_DIRS}
        # We don't use the "include" directory, so specify the current directory to capture all the files
        ./
)

file(GLOB PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.pb.cc")
