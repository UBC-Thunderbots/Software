
# set the name of the binary file without the file extension
set(ROBOT_BINARY_NAME "robot_firmware")

# get all of the source files
file(GLOB_RECURSE firmware-src
        "${FIRMWARE_SOURCE_DIR}/usb/*.c"
        "${FIRMWARE_SOURCE_DIR}/main/*.c"
        "${FIRMWARE_SOURCE_DIR}/stm32lib/*.c"
        "${FIRMWARE_SOURCE_DIR}/cdcacm/*.c"
        "${FIRMWARE_SOURCE_DIR}/freertos/source/*.c"
        "${FIRMWARE_SOURCE_DIR}/main/*.h"
        "${SHARED_UTIL_SRC}"
        )

# add robot firmware specific include directories
include_directories("${FIRMWARE_SOURCE_DIR}/main")
include_directories("${FIRMWARE_SOURCE_DIR}/cdcacm/include")

# creates the .bin, .elf, and .map files
# this function is included in firmware/CMakeLists.txt which this CMakeLists.txt file is a subdirectory of
build_firmware("${FIRMWARE_SOURCE_DIR}" "${firmware-src}" "${ROBOT_BINARY_NAME}" "main")

# path to the compiled file without the file extensions
# BINARY_NAME was set in the build_firmware function
set(FULL_BIN_PATH ${FIRMWARE_SOURCE_DIR}/main/${ROBOT_BINARY_NAME})


##### BUILD FOR DFU #####
add_custom_target(${ROBOT_BINARY_NAME}.dfu)
# copy the .bin file to a .dfu file for flashing
# don't put double quotes around the commands!
add_custom_command(
        TARGET "${ROBOT_BINARY_NAME}.dfu"
        POST_BUILD
        COMMAND cp ${FULL_BIN_PATH}.bin ${FULL_BIN_PATH}.dfu
        COMMAND dfu-util -a Firmware -D ${FULL_BIN_PATH}.dfu)

# add a dependency so that the build_firmware gets called to build firmware before flashing
add_dependencies(${ROBOT_BINARY_NAME}.dfu "${ROBOT_BINARY_NAME}.elf")

##### BUILD FOR DFUSE #####
add_custom_target(${ROBOT_BINARY_NAME}.dfuse)
include("${FIRMWARE_SOURCE_DIR}/cmake_binary_builders/dfu.cmake")
flash("${ROBOT_BINARY_NAME}" "dfuse")
