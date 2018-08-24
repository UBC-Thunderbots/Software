# This project name must match the name of the package defined in package.xml
project(thunderbots)

# Build in "Release" (with lots of compiler optimizations) by default
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY VALUE "Release")
endif()

# For external editor code completion (Visual Studio Code, Vim, etc.)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
set(CMAKE_CXX_FLAGS "-Wnon-virtual-dtor")


#############################################
## Find and include pacakges and libraries ##
#############################################

# Call our CMake file to include and build protobuf
include("${CMAKE_CURRENT_SOURCE_DIR}/proto/build_proto.cmake")

# Create a list of files from the shared folder
file(GLOB_RECURSE SHARED_UTIL_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/*.c
        ${CMAKE_CURRENT_SOURCE_DIR}/../shared/*.h
        )


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
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/..
        )

# Find all the generated protobuf source files
file(GLOB PROTO_SRCS "${CMAKE_CURRENT_SOURCE_DIR}/proto/*.pb.cc")

file(GLOB_RECURSE BACKEND_INPUT_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/backend_input/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/backend_input/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/geom/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/team.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp
        )
add_executable (backend_input
        ${PROTO_SRCS}
        ${BACKEND_INPUT_SRC}
        ${SHARED_UTIL_SRC}
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(backend_input ${catkin_EXPORTED_TARGETS})
target_link_libraries(backend_input
        ${catkin_LIBRARIES}
        ${PROTOBUF_LIBRARIES}
        )

file(GLOB_RECURSE AI_LOGIC_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/*.h
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
target_link_libraries(ai_logic ${catkin_LIBRARIES})

file(GLOB_RECURSE BACKEND_OUTPUT_SRC LIST_DIRECTORIES false CONFIGURE_DEPENDS
        ${CMAKE_CURRENT_SOURCE_DIR}/backend_output/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/backend_output/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/primitive/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/primitive/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/ai/world/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.h
        ${CMAKE_CURRENT_SOURCE_DIR}/util/*.cpp
        )
add_executable (backend_output
        ${PROTO_SRCS}
        ${BACKEND_OUTPUT_SRC}
        ${SHARED_UTIL_SRC}
        )
# Depend on exported targets (other packages) so that the messages in our thunderbots_msgs package are built first.
# This way the message headers are always generated before they are used in compilation here.
add_dependencies(backend_output ${catkin_EXPORTED_TARGETS})
target_link_libraries(backend_output ${catkin_LIBRARIES}
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
            test/field/field.cpp
            ai/world/field.cpp
            )
    target_link_libraries(field_test ${catkin_LIBRARIES})

    catkin_add_gtest(grsim_backend_test
            ${PROTO_SRCS}
            test/grsim_backend/grsim_backend.cpp
            backend_output/grsim/grsim_backend.cpp
            backend_output/grsim/grsim_backend.h
            backend_output/backend.h
            )
    target_link_libraries(grsim_backend_test ${catkin_LIBRARIES}
            ${PROTOBUF_LIBRARIES}
            )
endif()
