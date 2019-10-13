#!/bin/bash

CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
THIS_SCRIPT_FILENAME=`basename "$0"`

NUM_ARGS=$#
if [ $NUM_ARGS -ne 2 ]; then
    echo "Error: Incorrect number of arguments to script"
    echo "Usage: $THIS_SCRIPT_FILENAME /path/to/something.elf /path/to/some_openocd_config.cfg"
    exit 1
fi

# Start openocd as a child process
echo $2
openocd -f "$2" &

# Run gdb and connect to openocd
arm-none-eabi-gdb \
    -ex "target extended-remote :3333" \
    -ex "directory $CURR_DIR/../.." \
    -tui
