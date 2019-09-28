
function(set_arm_compiler)
    # tell the compiler we are using linux in the parent scope, which is required by the arm toolchains
    set(CMAKE_SYSTEM_NAME "Linux" PARENT_SCOPE)
    # set the path to the arm-none-eabi. This must match the setup_firmware.sh script
    set(ARM_CC_LOCATION "/opt/tbots-arm-toolchain/bin/arm-none-eabi-gcc")
    set(ARM_OBJCOPY_LOCATION "/opt/tbots-arm-toolchain/bin/arm-none-eabi-objcopy")
    # check if the files were built/installed
    if (NOT EXISTS "${ARM_CC_LOCATION}")
        message(SEND_ERROR "Could not find arm compiler at ${ARM_CC_LOCATION}. See the wiki to make sure you have properly setup the ARM toolchains.")
    endif()
    if (NOT EXISTS "${ARM_OBJCOPY_LOCATION}")
        message(SEND_ERROR "Could not find arm objcopy at ${ARM_OBJCOPY_LOCATION}. See the wiki to make sure you have properly setup the ARM toolchains.")
    endif()
    # setting in the parent scope is equivalent to returning a variable
    # set the compiler to use arm-none-eabi-gcc in the parent scope
    set(CMAKE_C_COMPILER "${ARM_CC_LOCATION}" PARENT_SCOPE)
    # set the object copier which is used to make the .bin file in the parent scope
    set(CMAKE_OBJCOPY "${ARM_OBJCOPY_LOCATION}" PARENT_SCOPE)
endfunction(set_arm_compiler)