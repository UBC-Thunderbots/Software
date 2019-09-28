
# a common function for the dongle and robot to use to flash the robot that adds a dependency on compiling binaries
# before flashing happens
# the target is the binary to flash without the extension, and the extension is one of [dfu, dfuse]
function(flash target extension)
    # copy the .bin file to a .dfu file for flashing
    # don't put double quotes around the commands!
    add_custom_command(
            TARGET "${target}.${extension}"
            POST_BUILD
            COMMAND dfu-util
            -d 0483
            -a "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg"
            -D ${FULL_BIN_PATH}.bin
            -s 0x08000000:leave)


    # add a dependency so that the build_firmware gets called to build firmware before flashing
    add_dependencies(${target}.${extension} "${target}.elf")
endfunction(flash)