#!/bin/bash
#
# This script allows the user to automatically generate cubemx code into our filestructure
#
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

# The name of this script
THIS_SCRIPT_FILENAME=$(basename "$0")

# the root directory of this project (ie. the location of the workspace file). this is a
# bit of a hack, but it works.
WORKSPACE_DIR="$CURR_DIR/../../"
TEMP_DIR="/tmp/tbots_autogen"

# cleanup temp dir
mkdir -p $TEMP_DIR/Src
mkdir -p $TEMP_DIR/Inc
rm -rf $TEMP_DIR/Src/*
rm -rf $TEMP_DIR/Inc/*

PATH_TO_IOC="__main__/firmware_new/boards/frankie_v1/frankie.ioc"

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
echo $PATH_TO_IOC
echo $TEMP_DIR
printf "$CUBE_SCRIPT" $(rlocation $PATH_TO_IOC) $TEMP_DIR > regen.stm32cube.script
/opt/STM32CubeMX/STM32CubeMX -s $TEMP_DIR/regen.stm32cube.script
