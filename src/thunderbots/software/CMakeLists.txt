# This project name must match the name of the package defined in package.xml
project(thunderbots)

# Build in "Release" (with lots of compiler optimizations) by default
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

# For external editor code completion (Visual Studio Code, Vim, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use PkgConfig to find some packages using pkg_check_modules
find_package(PkgConfig REQUIRED)
include(FindPkgConfig)

####################
## Compiler Flags ##
####################

### Set cmake standard to c++ 17 ###
set(CMAKE_CXX_STANDARD 17)

### Set compiler flags for all code ###
add_definitions(-Wno-deprecated) # Don't warn about including "old" headers
add_definitions(-fno-common) # Do not allow multiple definitions of the same global variable
add_definitions(-Werror) # Treat warnings as errors, so they don't build up

### Set compiler flags only for cpp code. These flags do not work for C files ###
# Warn if parent classes do not define a virtual destructor
# See https://www.quantstart.com/articles/C-Virtual-Destructors-How-to-Avoid-Memory-Leaks
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wnon-virtual-dtor")


#############################################
## Find and include pacakges and libraries ##
#############################################

# Call our CMake file to include and build protobuf
include("${CMAKE_CURRENT_SOURCE_DIR}/proto/build_proto.cmake")

# Create a list of files from the shared folder
file(GLOB SHARED_UTIL_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/*.h
        )

# Find the g3log library, the logger we use for our AI
# See: https://github.com/KjellKod/g3log
SET(G3_LIBRARY_PATH "/usr/local/include")
message("g3log lib path: ${G3_LIBRARY_PATH}")
find_library(G3LOG g3logger PATHS ${G3_LIBRARY_PATH})
if(NOT G3LOG OR NOT EXISTS "${G3LOG}")
    message(FATAL_ERROR "G3LOG library not found")
endif()
message("g3log path: ${G3LOG}")

# Find boost components
find_package(Boost REQUIRED COMPONENTS
        coroutine
        )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        message_generation
        thunderbots_msgs
        dynamic_reconfigure
        )

## Specify the configuration file for dynamic_reconfigure pkg
generate_dynamic_reconfigure_options(
    ${CMAKE_CURRENT_SOURCE_DIR}/param_server/cfg/Params.cfg
)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES thunderbots
        CATKIN_DEPENDS roscpp std_msgs dynamic_reconfigure
        DEPENDS message_runtime
        )

###########
## Build ##
###########

# Specify additional locations of header files
include_directories(SYSTEM
	${catkin_INCLUDE_DIRS}
	)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PROTOBUF_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

# Find all the generated protobuf source files
file(GLOB PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.pb.cc")

file(GLOB_RECURSE NETWORK_INPUT_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/network_input/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/network_input/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp
        )
add_executable (network_input
        ${PROTO_SRCS}
        ${NETWORK_INPUT_SRC}
        ${SHARED_UTIL_SRC}
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(network_input ${catkin_EXPORTED_TARGETS})
target_link_libraries(network_input
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        ${G3LOG}
        )

file(GLOB_RECURSE AI_LOGIC_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp
        )
add_executable (ai_logic
        ${AI_LOGIC_SRC}
        ${SHARED_UTIL_SRC}
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(ai_logic ${catkin_EXPORTED_TARGETS})
target_link_libraries(ai_logic ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        ${G3LOG}
        )

file(GLOB_RECURSE GRSIM_COMMUNICATION_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/grsim_communication/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/grsim_communication/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/primitive/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/primitive/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp
        )
add_executable (grsim_communication
        ${PROTO_SRCS}
        ${GRSIM_COMMUNICATION_SRC}
        ${SHARED_UTIL_SRC}
        )

# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(grsim_communication ${catkin_EXPORTED_TARGETS})
target_link_libraries(grsim_communication ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        ${G3LOG}
        )

file(GLOB_RECURSE RADIO_COMMUNICATION_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/radio_communication/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/radio_communication/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/primitive/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/primitive/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp
        )
add_executable (radio_communication
        ${RADIO_COMMUNICATION_SRC}
        ${SHARED_UTIL_SRC}
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(radio_communication ${catkin_EXPORTED_TARGETS})

##### LIBUSB #####
pkg_search_module("LIBUSB_1" REQUIRED "libusb-1.0")
include_directories(SYSTEM "${LIBUSB_1_INCLUDE_DIRS}")
link_directories("${LIBUSB_1_LIBRARY_DIRS}")
add_definitions("${LIBUSB_1_DEFINITIONS}")

##### SIGC++  ######
pkg_search_module("SIGC" REQUIRED "sigc++-2.0")
include_directories(SYSTEM "${SIGC_INCLUDE_DIRS}")
link_directories("${SIGC_LIBRARY_DIRS}")

target_link_libraries(radio_communication ${catkin_LIBRARIES}
        ${G3LOG}
        ${LIBUSB_1_LIBRARIES}
        ${SIGC_LIBRARIES}
        )

file(GLOB_RECURSE PARAM_SERVER_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/param_server/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/param_server/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/util/parameter/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/parameter/*.cpp
        )
add_executable (parameters
        ${PROTO_SRCS}
        ${PARAM_SERVER_SRC}
        ${SHARED_UTIL_SRC}
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(parameters thunderbots_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(parameters  ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        )

#############
## Testing ##
#############

###### IMPORTANT #####
# DO NOT USE GLOBBING FOR TEST EXECUTABLES. SPECIFY EACH SOURCE FILE.
# This is to make sure we know exactly what files/interfaces are being tested and that we are not accidentally
# picking up other files that may affect test behaviour. Unit tests should not need too many files to be specified
# and we want to make sure the minimum set of files required for the test are being used.

##### Unit Tests #####

## Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
    #gtest
    catkin_add_gtest(geom_angle_test
            test/geom/angle.cpp
            geom/angle.h
            )
    target_link_libraries(geom_angle_test ${catkin_LIBRARIES})

    catkin_add_gtest(geom_point_test
            test/geom/point.cpp
            geom/point.h
            )
    target_link_libraries(geom_point_test ${catkin_LIBRARIES})

    catkin_add_gtest(field_test
            test/world/field.cpp
            ai/world/field.cpp
            util/parameter/dynamic_parameters.cpp
            )
    target_link_libraries(field_test ${catkin_LIBRARIES})

    catkin_add_gtest(grsim_backend_test
            ${PROTO_SRCS}
            test/grsim_backend/grsim_backend.cpp
            grsim_communication/motion_controller/motion_controller.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/world/robot.cpp
            ai/world/team.cpp
            ai/world/ball.cpp
            grsim_communication/grsim_backend.cpp
            grsim_communication/grsim_backend.h
            grsim_communication/grsim_backend.cpp
            grsim_communication/visitor/grsim_command_primitive_visitor.cpp
            util/timestamp.cpp
            )
    target_link_libraries(grsim_backend_test ${catkin_LIBRARIES}
            ${PROTOBUF_LIBRARIES}
            ${G3LOG}
            )

    catkin_add_gtest(geom_util_test
            test/geom/util.cpp
            geom/util.cpp
            geom/util.h
            geom/point.h
            geom/angle.h
            geom/segment.h
            geom/ray.h
            geom/circle.h
            geom/line.h)

    target_link_libraries(geom_util_test ${catkin_LIBRARIES})

    catkin_add_gtest(shared_util_test
            ../shared/test/util.cpp
            ../shared/util.c
            ../shared/util.h)

    target_link_libraries(shared_util_test ${catkin_LIBRARIES})

    catkin_add_gtest(ball_test
            test/world/ball.cpp
            ai/world/ball.cpp
            util/timestamp.cpp
            geom/point.h
            geom/angle.h)

    target_link_libraries(ball_test ${catkin_LIBRARIES})

    catkin_add_gtest(robot_test
            test/world/robot.cpp
            ai/world/robot.cpp
            util/timestamp.cpp
            geom/point.h
            geom/angle.h)

    target_link_libraries(robot_test ${catkin_LIBRARIES})

    catkin_add_gtest(team_test
            test/world/team.cpp
            ai/world/team.cpp
            ai/world/robot.cpp
            util/timestamp.cpp
            geom/point.h
            geom/angle.h)

    target_link_libraries(team_test ${catkin_LIBRARIES})

    catkin_add_gtest(ros_message_util_test
            test/util/ros_messages.cpp
            util/ros_messages.cpp
            ai/world/ball.cpp
            ai/world/robot.cpp
            ai/world/field.cpp
            ai/world/team.cpp
            util/timestamp.cpp
            geom/point.h
            geom/angle.h)

    target_link_libraries(ros_message_util_test ${catkin_LIBRARIES})

    catkin_add_gtest(motion_controller_test
            test/grsim_backend/motion_controller.cpp
            grsim_communication/motion_controller/motion_controller.cpp
            ai/world/robot.cpp
            geom/point.h
            geom/angle.h
            util/timestamp.cpp)

    target_link_libraries(motion_controller_test ${catkin_LIBRARIES})

    catkin_add_gtest(world_test
            test/world/world.cpp
            test/test_util/test_util.cpp
            ai/world/world.cpp
            ai/world/ball.cpp
            ai/world/field.cpp
            ai/world/robot.cpp
            ai/world/team.cpp
            ai/world/game_state.cpp
            util/timestamp.cpp
            util/parameter/dynamic_parameters.cpp
            )

    target_link_libraries(world_test ${catkin_LIBRARIES})

    catkin_add_gtest(test_util_test
            test/test_util/test_util_test.cpp
            test/test_util/test_util.cpp
            ai/world/world.cpp
            ai/world/ball.cpp
            ai/world/field.cpp
            ai/world/robot.cpp
            ai/world/team.cpp
            ai/world/game_state.cpp
            util/timestamp.cpp
            util/parameter/dynamic_parameters.cpp
            )

    target_link_libraries(test_util_test ${catkin_LIBRARIES})

    catkin_add_gtest(game_state_test
            test/world/game_state.cpp
            ai/world/game_state.cpp
            test/test_util/test_util.cpp
            ai/world/world.cpp
            ai/world/ball.cpp
            ai/world/field.cpp
            ai/world/robot.cpp
            ai/world/team.cpp
            ai/world/game_state.cpp
            util/refbox_constants.cpp
            util/timestamp.cpp
            util/parameter/dynamic_parameters.cpp
            )

    target_link_libraries(game_state_test ${catkin_LIBRARIES})

    catkin_add_gtest(move_spin_primitive_test
            test/grsim_command_primitive_visitor/move_spin_primitive.cpp
            grsim_communication/visitor/grsim_command_primitive_visitor.cpp
            util/ros_messages.cpp
            ai/world/ball.cpp
            test/test_util/test_util.cpp
            grsim_communication/motion_controller/motion_controller.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/primitive.cpp
            geom/point.h
            ai/world/world.cpp
            ai/world/field.cpp
            ai/world/robot.cpp
            ai/world/team.cpp
            ai/world/game_state.cpp
            geom/angle.h
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            util/timestamp.cpp
            util/parameter/dynamic_parameters.cpp
            )
    target_link_libraries(move_spin_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(primitive_test
            test/primitive/primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(move_primitive_test
            test/primitive/move_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(move_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(movespin_primitive_test
            test/primitive/movespin_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(movespin_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(directwheels_primitive_test
            test/primitive/directwheels_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(directwheels_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(pivot_primitive_test
            test/primitive/pivot_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(pivot_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(chip_primitive_test
            test/primitive/chip_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(chip_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(kick_primitive_test
            test/primitive/kick_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(kick_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(direct_velocity_primitive_test
            test/primitive/direct_velocity_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(direct_velocity_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(catch_primitive_test
            test/primitive/catch_primitive.cpp
            ai/primitive/primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/stop_primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(catch_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(stop_primitive_test
            test/primitive/stop_primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(stop_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(dribble_primitive_test
            test/primitive/dribble_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/primitive.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            )

    target_link_libraries(dribble_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(grsim_catch_primitive_test
            grsim_communication/visitor/grsim_command_primitive_visitor.cpp
            test/grsim_command_primitive_visitor/catch_primitive.cpp
            util/ros_messages.cpp
            ai/world/ball.cpp
            test/test_util/test_util.cpp
            grsim_communication/motion_controller/motion_controller.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/primitive.cpp
            geom/point.h
            ai/world/world.cpp
            ai/world/field.cpp
            ai/world/robot.cpp
            ai/world/team.cpp
            ai/world/game_state.cpp
            geom/angle.h
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            util/timestamp.cpp
            util/parameter/dynamic_parameters.cpp
    )

    target_link_libraries(grsim_catch_primitive_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(mrf_primitive_visitor_test
            test/radio_communication/visitor/mrf_primitive_visitor.cpp
            radio_communication/visitor/mrf_primitive_visitor.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/primitive.cpp
            )
    target_link_libraries(mrf_primitive_visitor_test ${catkin_LIBRARIES}
            ${G3LOG})

    catkin_add_gtest(timestamp_test
            test/util/timestamp.cpp
            util/timestamp.cpp
            )

    target_link_libraries(timestamp_test ${catkin_LIBRARIES})

    catkin_add_gtest(deflect_off_enemy_target_test
            test/evaluation/deflect_off_enemy_target.cpp
            ai/hl/stp/evaluation/deflect_off_enemy_target.cpp
            ai/hl/stp/evaluation/deflect_off_enemy_target.h
            geom/util.cpp
            util/timestamp.cpp
            ai/world/ball.cpp
            ai/world/field.cpp
            ai/world/game_state.cpp
            ai/world/robot.cpp
            ai/world/team.cpp
            ai/world/world.cpp
            test/test_util/test_util.cpp
            util/parameter/dynamic_parameters.cpp
            )

    target_link_libraries(deflect_off_enemy_target_test ${catkin_LIBRARIES})

    catkin_add_gtest(evaluation_robot_orientation_within_angle_threshold_of_target_test
            test/evaluation/robot.cpp
            ai/hl/stp/evaluation/robot.cpp
            util/parameter/dynamic_parameters.cpp
            )

    target_link_libraries(evaluation_robot_orientation_within_angle_threshold_of_target_test ${catkin_LIBRARIES})

    catkin_add_gtest(evaluation_nearest_friendly_robot_test
    test/evaluation/team.cpp
    ai/hl/stp/evaluation/team.cpp
    ai/world/team.cpp
    ai/world/robot.cpp
    util/timestamp.cpp
    geom/point.h
    geom/angle.h
    )
    target_link_libraries(evaluation_nearest_friendly_robot_test ${catkin_LIBRARIES})

    catkin_add_gtest(move_action_test
            test/action/move_action.cpp
            ai/hl/stp/action/action.cpp
            ai/hl/stp/action/move_action.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/primitive.cpp
            ai/intent/move_intent.cpp
            ai/intent/intent.cpp
            ai/world/robot.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            util/timestamp.cpp
            )
    target_link_libraries(move_action_test
            ${catkin_LIBRARIES}
            ${G3LOG}
            ${Boost_LIBRARIES})

    catkin_add_gtest(move_tactic_test
            test/tactic/move_tactic.cpp
            test/test_util/test_util.cpp
            ai/hl/stp/tactic/move_tactic.cpp
            ai/hl/stp/tactic/tactic.cpp
            ai/hl/stp/action/move_action.cpp
            ai/hl/stp/action/action.cpp
            ai/primitive/catch_primitive.cpp
            ai/primitive/chip_primitive.cpp
            ai/primitive/direct_velocity_primitive.cpp
            ai/primitive/dribble_primitive.cpp
            ai/primitive/kick_primitive.cpp
            ai/primitive/pivot_primitive.cpp
            ai/primitive/move_primitive.cpp
            ai/primitive/movespin_primitive.cpp
            ai/primitive/stop_primitive.cpp
            ai/primitive/directwheels_primitive.cpp
            ai/primitive/primitive.cpp
            ai/intent/move_intent.cpp
            ai/intent/intent.cpp
            ai/world/robot.cpp
            ai/world/ball.cpp
            ai/world/field.cpp
            ai/world/team.cpp
            ai/world/game_state.cpp
            ai/world/world.cpp
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            util/timestamp.cpp
            util/parameter/dynamic_parameters.cpp
            )
    target_link_libraries(move_tactic_test
            ${catkin_LIBRARIES}
            ${G3LOG}
            ${Boost_LIBRARIES})
endif()

##### ROSTests / Integration Tests #####

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    # The test node that verifies the logger is working correctly
    add_rostest_gtest(logger_test test/util/logger_test.test
            util/logger/custom_g3log_sinks.h
            util/logger/init.h
            test/util/logger_test.cpp
            util/timestamp.cpp
            )
    target_link_libraries(logger_test ${catkin_LIBRARIES}
            ${G3LOG})

    # The test node that verifies the rostest utils are working correctly
    add_rostest_gtest(rostest_util_test test/test_util/rostest_util_test.test
            test/test_util/rostest_util_test.cpp
            util/timestamp.cpp
            )
    target_link_libraries(rostest_util_test ${catkin_LIBRARIES})

     # The test node that verifies the dynamic_parameters are working
    add_rostest_gtest(dynamic_parameters_test test/util/rostest_dynamic_parameters.test
            test/util/test_dynamic_parameters.cpp
            util/parameter/dynamic_parameters.cpp
            )
    target_link_libraries(dynamic_parameters_test ${catkin_LIBRARIES})

     # The test node that verifies the dynamic_parameters exist in the parameter server
    add_rostest_gtest(check_parameter_existance_test test/util/rostest_parameter_exists.test
            test/util/test_parameter_exists.cpp
            util/parameter/dynamic_parameters.cpp
            )
    target_link_libraries(check_parameter_existance_test ${catkin_LIBRARIES})

endif()
