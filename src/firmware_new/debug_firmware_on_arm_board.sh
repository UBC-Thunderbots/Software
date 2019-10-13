#!/bin/bash

CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
THIS_SCRIPT_FILENAME=`basename "$0"`

# We require exactly one argument, the path to the `.elf`
if [$# -ne 2]; then
    echo "Usage: ./$THIS_SCRIPT_FILENAME /path/to/something.elf /path/to/some_openocd_config.cfg"
    exit 1
fi

# Start openocd

# Start gdb to connect to the board via openocd
# TODO: the directory here needs to be set relatively
arm-none-eabi-gdb -ex "directory /home/gareth/programming/thunderbots/Software/src/" "$@"
