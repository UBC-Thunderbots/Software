#!/bin/bash
#
# This script allows the user to automatically generate cubemx
# code into our filestructure
CUBE_VERSION=5.6.1

# The directory this script is in
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")

# the root directory of this project (ie. the location of the workspace file). this is a
# bit of a hack, but it works.
WORKSPACE_DIR="$CURR_DIR/../../"
THIS_SCRIPT_FILENAME=$(basename "$0")

TEMP_DIR="/tmp/tbots_autogen"
CUBE_EXECUTABLE="/opt/STM32CubeMX_$CUBE_VERSION/STM32CubeMX"

# cleanup temp dir and create Src/Inc folder, the way Cube expects this output
mkdir -p $TEMP_DIR/Src
mkdir -p $TEMP_DIR/Inc

# make sure we got an argument
NUM_ARGS=$#
if [ $NUM_ARGS -ne 1 ]; then
    echo "Error: Incorrect number of arguments to script"
    echo "Usage: $THIS_SCRIPT_FILENAME path/to/folder/with/.ioc/file/"
    exit 1
fi

FIRMWARE_DIR=$WORKSPACE_DIR$1

# check if this is a valid path
if [ ! -d $FIRMWARE_DIR ]; then
    echo "Error: $1 is not a valid directory."
    echo "Make sure the path is relative to the WORKSPACE file"
    exit 1
fi

# make sure we only have 1 ioc file
IOC_FILES=(`find $FIRMWARE_DIR -maxdepth 1 -name "*.ioc"`)
IOC_FILE="invalid"

if [ ${#IOC_FILES[@]} -gt 1 ]; then 
    echo "Error: More than 1 ioc file found in directory"
    exit 1
elif [ ${#IOC_FILES[@]} -lt 1 ]; then 
    echo "Error: No ioc file found in provided directory"
    exit 1
else 
    IOC_FILE=${IOC_FILES[0]}
    echo "Using .ioc file found here: $IOC_FILE" 
fi

# copy our current files here
cp $FIRMWARE_DIR/*.c $TEMP_DIR/Src/
cp $FIRMWARE_DIR/*.h $TEMP_DIR/Inc/

CUBE_SCRIPT='''
# Load the project-specific config file (.ioc)
config load %s

# Generate the peripheral initializations in separated files
project couplefilesbyip 1

# Generate code in the project directory
generate code %s

# Exit the program
exit
'''
cd $TEMP_DIR

# create the code generation cube script
printf "$CUBE_SCRIPT" $IOC_FILE $TEMP_DIR > regen.stm32cube.script

# generate code
$CUBE_EXECUTABLE -s $TEMP_DIR/regen.stm32cube.script

# the stm32h7xx_hal_conf.h file doesn't play well with clang-format, so disable formatting
# the file does not have any "USER-CODE" sections, so this won't cause any problems
echo "$(echo '// clang-format off'; cat ./Inc/stm32h7xx_hal_conf.h; echo '// clang-format on')" > ./Inc/stm32h7xx_hal_conf.h

# move the generated files to the right path
mv $TEMP_DIR/Src/*.c $FIRMWARE_DIR/
mv $TEMP_DIR/Inc/*.h $FIRMWARE_DIR/
