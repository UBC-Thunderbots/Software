# set the minimum cmake version
cmake_minimum_required(VERSION 3.7)

# create the firmware project so that you can open clion from the firmware directory
project(firmware)

# add our testing framework, which must be done before the arm compiler is set because it builds an x86 executable
add_subdirectory(test)

# set the location of our firmware directory, which both dongle and robot firmware rely on in their respective
# CMakeLists.txt files
set(FIRMWARE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# IMPORTANT: Compiler must be set here so it can be used by dongle and robot firmware
include("${FIRMWARE_SOURCE_DIR}/cmake_binary_builders/arm_compiler.cmake")
set_arm_compiler()

# add the subdirectories for dongle and main which must be added after the arm compiler is set
add_subdirectory(dongle)
add_subdirectory(main)
