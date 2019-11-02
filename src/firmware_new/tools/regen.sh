#!/bin/bash
#
# This script allows the user to automatically generate cubemx code into our filestructure

# --- begin runfiles.bash initialization ---
# Copy-pasted from Bazel's Bash runfiles library (tools/bash/runfiles/runfiles.bash).
# See here for why we need this:
# https://github.com/bazelbuild/bazel/blob/8fa6b3fe71f91aac73c222d8082e75c69d814fa7/tools/bash/runfiles/runfiles.bash#L38-L59
set -euo pipefail
if [[ ! -d "${RUNFILES_DIR:-/dev/null}" && ! -f "${RUNFILES_MANIFEST_FILE:-/dev/null}" ]]; then
  if [[ -f "$0.runfiles_manifest" ]]; then
    export RUNFILES_MANIFEST_FILE="$0.runfiles_manifest"
  elif [[ -f "$0.runfiles/MANIFEST" ]]; then
    export RUNFILES_MANIFEST_FILE="$0.runfiles/MANIFEST"
  elif [[ -f "$0.runfiles/bazel_tools/tools/bash/runfiles/runfiles.bash" ]]; then
    export RUNFILES_DIR="$0.runfiles"
  fi
fi
if [[ -f "${RUNFILES_DIR:-/dev/null}/bazel_tools/tools/bash/runfiles/runfiles.bash" ]]; then
  source "${RUNFILES_DIR}/bazel_tools/tools/bash/runfiles/runfiles.bash"
elif [[ -f "${RUNFILES_MANIFEST_FILE:-/dev/null}" ]]; then
  source "$(grep -m1 "^bazel_tools/tools/bash/runfiles/runfiles.bash " \
    "$RUNFILES_MANIFEST_FILE" | cut -d ' ' -f 2-)"
else
  echo >&2 "ERROR: cannot find @bazel_tools//tools/bash/runfiles:runfiles.bash"
  exit 1
fi
set +e
# --- end runfiles.bash initialization ---

# The directory this script is in
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")

# the root directory of this project (ie. the location of the workspace file). this is a
# bit of a hack, but it works.
WORKSPACE_DIR="$CURR_DIR/../../"
THIS_SCRIPT_FILENAME=$(basename "$0")

TEMP_DIR="/tmp/tbots_autogen"

# cleanup temp dir and create Src/Inc folder, the way Cube expects this output
mkdir -p $TEMP_DIR/Src
mkdir -p $TEMP_DIR/Inc
rm -rf $TEMP_DIR/Src/*
rm -rf $TEMP_DIR/Inc/*

# make sure we got an argument
NUM_ARGS=$#
if [ $NUM_ARGS -ne 1 ]; then
    echo "Error: Incorrect number of arguments to script"
    echo "Usage: $THIS_SCRIPT_FILENAME firmware_new/boards/frankie_v1"
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
    echo "Using .ioc file found here: $IOC_FILE" 
    IOC_FILE=${IOC_FILES[0]}
fi

# copy our current files here
cp $FIRMWARE_DIR/*.c $TEMP_DIR/Src/
cp $FIRMWARE_DIR/*.h $TEMP_DIR/Inc/

CUBE_SCRIPT='''
# Load the project-specific config file (.ioc)
config load %s

# Generate the peripheral initializations in main.c
project couplefilesbyip 0

# Generate code in the project direcotry
generate code %s

# Exit the program
exit
'''
cd $TEMP_DIR

# create the code generation cube script
printf "$CUBE_SCRIPT" $IOC_FILE $TEMP_DIR > regen.stm32cube.script

# generate code
/opt/STM32CubeMX/STM32CubeMX -s $TEMP_DIR/regen.stm32cube.script

# move the generated files to the right path
mv $TEMP_DIR/Src/*.c $FIRMWARE_DIR/
mv $TEMP_DIR/Inc/*.h $FIRMWARE_DIR/

