
# use this function to set the include directories for all of the header files.
# we want this in its own functino so we can reuse it for unit tests.
function(set_include_dirs FIRMWARE_SOURCE_DIR)
    include_directories("${FIRMWARE_SOURCE_DIR}")
    include_directories("${FIRMWARE_SOURCE_DIR}/stm32lib/include")
    include_directories("${FIRMWARE_SOURCE_DIR}/freertos/include")
    include_directories("${FIRMWARE_SOURCE_DIR}/usb/include")
endfunction(set_include_dirs)


# call this function to build the .bin, .elf, and .map files
function(build_firmware FIRMWARE_SOURCE_DIR SRC_FILES BINARY_NAME OUT_DIR)
    # specify where all of our headers are
    set_include_dirs(${FIRMWARE_SOURCE_DIR})
    # add preprocessor definitions
    add_definitions(-DSTM32LIB_USE_FREERTOS)

    # add the source files to our executable
    add_executable(${BINARY_NAME}.elf "${SRC_FILES}")

    # a list of the c flags we are going to use
    set(CFLAGS
        -std=gnu99
        -O2
        -mfloat-abi=hard
        -mlittle-endian
        -mcpu=cortex-m4
        -mfpu=fpv4-sp-d16
        -mthumb
        -ggdb3
        -fno-common
        -ffunction-sections
        -static
        -Wall
        -Wextra
        -Wdouble-promotion
        -Wpointer-arith)
        
    # specify all of the compiler flags
    target_compile_options(${BINARY_NAME}.elf PUBLIC ${CFLAGS})

    # specify the flags that will be passed to the linker
    # all of the flags must be on one line otherwise the code won't build
    set(LINKER_FLAGS "-Wl,--gc-sections -Wl,--build-id=none ")
    # put all of the C flags into the linker flags
    # the formatting is different for the linker flags from the cflags. Each flag needs to be separated by a space,
    # whereas for the cflags you need to use a cmake list
    foreach(flag ${CFLAGS})
        set(LINKER_FLAGS "${LINKER_FLAGS} ${flag}")
    endforeach(flag)
    # set our linker script
    set(LINKER_SCRIPT "${FIRMWARE_SOURCE_DIR}/stm32lib/stm32f405.ld")
    # add the linker script and the map option to our linker flags
    # the map options tells the linker where to put the bulit .map file
    set(LINKER_FLAGS "${LINKER_FLAGS} -Wl,-T${LINKER_SCRIPT} -Wl,-Map=${FIRMWARE_SOURCE_DIR}/${OUT_DIR}/${BINARY_NAME}.map")

    # add the linker flags and tell cmake to put our ${BINARY_NAME}.elf file in the specifed RUNTIME_OUTPUT_DIRECTORY
    set_target_properties(${BINARY_NAME}.elf
            PROPERTIES
            LINK_FLAGS ${LINKER_FLAGS}
            RUNTIME_OUTPUT_DIRECTORY ${FIRMWARE_SOURCE_DIR}/${OUT_DIR})


    # link against libraries
    # m is the math library that we want to link against (<math.h>)
    target_link_libraries(${BINARY_NAME}.elf "m")

    # tell cmake to copy the .elf file to the specified .bin file after compilation has finished
    set(BIN_FILE "${FIRMWARE_SOURCE_DIR}/${OUT_DIR}/${BINARY_NAME}.bin")
    add_custom_command(
            TARGET ${BINARY_NAME}.elf
            POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -Obinary $<TARGET_FILE:${BINARY_NAME}.elf> ${BIN_FILE}
            COMMENT "Building ${BIN_FILE}")

endfunction(build_firmware)