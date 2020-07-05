#!/bin/bash
#
# This script allows the user to automatically flash a `.bin` file to an MCU using
# DFU.
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

# The name of this script
THIS_SCRIPT_FILENAME=$(basename "$0")

# The number of arguments passed to this script
NUM_ARGS=$#

# Enable bazel runtime debugging
# RUNFILES_LIB_DEBUG=1

if [ $NUM_ARGS -ne 1 ]; then
  echo "Error: Incorrect number of arguments to script"
  echo "Usage: $THIS_SCRIPT_FILENAME radio_dongle or robot"
  exit 1
fi

if [[ "$1" == "radio_dongle" ]]; then
    echo "Flashing radio dongle!"
    bin_file="$(rlocation "__main__/firmware/boards/legacy_dongle_stm32f4/legacy_dongle_stm32f4.bin")"
elif [[ "$1" == "robot" ]]; then
    echo "Flashing robot!"
    bin_file="$(rlocation "__main__/firmware/boards/legacy_robot_stm32f4/legacy_robot_stm32f4.bin")"
else
  echo "Targets is chosen from:"
  echo "robot"
  echo "radio_dongle"
  exit 1
fi

# Try to flash the given binary
dfu-util -d 0483 -a "@Internal Flash  /0x08000000/04*016Kg,01*064Kg,07*128Kg" -D $bin_file  -s 0x08000000:leave
