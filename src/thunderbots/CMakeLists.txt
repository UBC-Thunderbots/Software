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

add_executable (backend_input
        ${PROTO_SRCS}
        backend_input/main.cpp
        backend_input/vision_client/netraw.cpp
        backend_input/vision_client/netraw.h
        backend_input/vision_client/netraw_util.h
        backend_input/vision_client/robocup_ssl_client.cpp
        backend_input/vision_client/robocup_ssl_client.h
        backend_input/message_util.cpp
        backend_input/message_util.h
        backend_input/filter/ball_filter.cpp
        backend_input/filter/ball_filter.h
        backend_input/filter/robot_filter.cpp
        backend_input/filter/robot_filter.h
        backend_input/filter/robot_team_filter.cpp
        backend_input/filter/robot_team_filter.h
        geom/point.h
        geom/angle.h
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(backend_input ${catkin_EXPORTED_TARGETS})
target_link_libraries(backend_input ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES})

add_executable (ai_logic
        ai/main.cpp
        ai/world/field.cpp
        ai/world/field.h
        ai/world/ball.cpp
        ai/world/ball.h
        ai/world/robot.cpp
        ai/world/robot.h
        ai/world/team.cpp
        ai/world/team.h
        ai/world/world.cpp
        ai/world/world.h
        ai/intent.cpp
        ai/intent.h
        ai/hl/hl.cpp
        ai/hl/hl.h
        ai/hl/stp/stphl.cpp
        ai/hl/stp/stphl.h
        ai/primitive/primitive.h
        ai/primitive/move_prim.cpp
        ai/primitive/move_prim.h
        ai/navigator/navigator.cpp
        ai/navigator/navigator.h
        ai/navigator/rrt/rrt.cpp
        ai/navigator/rrt/rrt.h
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(ai_logic ${catkin_EXPORTED_TARGETS})
target_link_libraries(ai_logic ${catkin_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    #gtest
    catkin_add_gtest(geom_angle_test
            geom/test/angle.cpp
            geom/angle.h
            )
    target_link_libraries(geom_angle_test ${catkin_LIBRARIES})

    catkin_add_gtest(geom_point_test
            geom/test/point.cpp
            geom/point.h
            )
    target_link_libraries(geom_point_test ${catkin_LIBRARIES})

    catkin_add_gtest(field_test
            ai/test/field.cpp
            ai/world/field.cpp
            )
    target_link_libraries(field_test ${catkin_LIBRARIES})
endif()
