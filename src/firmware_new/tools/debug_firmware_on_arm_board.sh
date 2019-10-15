#!/bin/bash

# The directory this script is located in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The name of this script
THIS_SCRIPT_FILENAME=`basename "$0"`

# The root of the bazel project, used to find source files for gdb
BAZEL_ROOT_DIR="$CURR_DIR/../.."

NUM_ARGS=$#
if [ $NUM_ARGS -ne 2 ]; then
    echo "Error: Incorrect number of arguments to script"
    echo "Usage: $THIS_SCRIPT_FILENAME /path/to/something.elf /path/to/some_openocd_config.cfg"
    exit 1
fi

OPENOCD_CFG_FILE=$2
ELF_FILE=$1

# Make sure target we're loading actually exists
if [ ! -f $ELF_FILE ]; then
    echo "File given does not exist: '$ELF_FILE'"
    exit 1
fi


# Stop openocd if it's already running
pkill openocd
KILLED_OPENOCD=$?

# If we killed openocd, wait for a second for the port to be free
if [ $KILLED_OPENOCD = 0 ]; then
    echo "Had to kill instance of openocd, waiting for a second for it to stop"
    sleep 2
fi

# Start openocd as a child process, and redirect all its output to /dev/null.
# If we don't redirect the output it will overwrite the gdb window
OPENOCD_RUN_CMD="openocd -f $OPENOCD_CFG_FILE"
eval "$OPENOCD_RUN_CMD > /dev/null 2>&1 &"

# Check that openocd is still running after a second. If it's not, that 
# probably means there was an issue starting it, so we exit
sleep 1
pgrep openocd 
OPENOCD_RUNNING=$?
if [ $OPENOCD_RUNNING -ne 0 ]; then
    echo "There was an issue running openocd, the command that failed was: '$OPENOCD_RUN_CMD'"
    exit 1
fi

# Run gdb, then:
#   - connect to openocd
#   - find all the source files so you can view them in gdb
#   - load firmware onto the board
arm-none-eabi-gdb \
    -ex "target extended-remote :3333" \
    -ex "directory $BAZEL_ROOT_DIR" \
    -ex "load" \
    -tui $ELF_FILE

# When gdb is finished, cleanup by stopping openocd
pkill openocd
