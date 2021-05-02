#!/bin/bash
#
# This script allows the user to automatically flash and debug a given binary to a
# ARM MCU over USB and debug it, all via OpenOCD
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

# The root directory of this project (ie. the location of the WORKSPACE file). This is a
# bit of a hack, but it works.
WORKSPACE_DIR="$CURR_DIR/../../"

# Enable bazel runtime debugging
#RUNFILES_LIB_DEBUG=1

# Load everything we need from bazel
path_to_arm_none_eabi_gdb=$(rlocation "__main__/external/arm_developer_gcc/bin/arm-none-eabi-gdb")
path_to_robot_stm32h7_elf=$(rlocation "__main__/firmware/boards/robot_stm32h7/robot_stm32h7_main")

# ADD NEW DEPENDENCIES HERE: it's used for error checking, you'll thank yourself later
EXPECTED_NUM_OF_BAZEL_DEPENDENCIES=2
bazel_dependencies=(
  $path_to_arm_none_eabi_gdb
  $path_to_robot_stm32h7_elf
)

# '.elf' files we want the user to be able to run
elf_files=(
  $path_to_robot_stm32h7_elf
)

# The correct board to give to 'openocd' for the given elf
declare -A elf_to_board_map=(
  [$path_to_robot_stm32h7_elf]="board/st_nucleo_h743zi.cfg"
)

# Make sure we successfully set everything
FOUND_NUM_OF_BAZEL_DEPENDENCIES=0
for dependency in "${bazel_dependencies[@]}"; do
  FOUND_NUM_OF_BAZEL_DEPENDENCIES=$((FOUND_NUM_OF_BAZEL_DEPENDENCIES + 1))
  if [[ ! -f "${dependency:-}" ]]; then
    echo >&2 "ERROR: could not look up path to given bazel dependency: $dependency"
    echo >&2 "Try enabling 'RUNFILES_LIB_DEBUG' in this script to see if there are any issues"
    exit 1
  fi
done
if [ "$FOUND_NUM_OF_BAZEL_DEPENDENCIES" -ne "$EXPECTED_NUM_OF_BAZEL_DEPENDENCIES" ]; then
  echo >&2 "ERROR: expected $EXPECTED_NUM_OF_BAZEL_DEPENDENCIES bazel dependencies, but only got $FOUND_NUM_OF_BAZEL_DEPENDENCIES"
  echo >&2 "Try enabling 'RUNFILES_LIB_DEBUG' in this script to see if there are any issues"
  exit 1
fi

# Check that the user selected an ELF file, and let them know what their options
# are if they have not
NUM_ARGS=$#
NUM_ELF_FILES="${#elf_files[@]}"
if [[ $NUM_ARGS -ne 1 ]] || [[ $1 =~ [^[:digit:]] ]] || [[ "$1" -ge "$NUM_ELF_FILES" ]]; then
  if [ $NUM_ARGS -ne 1 ]; then
    echo "Error: Incorrect number of arguments to script"
    echo "Usage: $THIS_SCRIPT_FILENAME INDEX_FOR_ELF_FILE"
  else
    echo "Error: Invalid index for ELF file given: $1"
  fi
  echo "INDEX_FOR_ELF_FILE is chosen from:"
  for i in "${!elf_files[@]}"; do
    printf "%s)  %s\n" "$i" "${elf_files[$i]}"
  done
  exit 1
fi
elf_file=${elf_files[$1]}

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
OPENOCD_CFG_FILE="${elf_to_board_map[$elf_file]}"
echo "Starting openocd with config $OPENOCD_CFG_FILE"
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
gdb_args=(
  -ex \"target extended-remote :3333\"
  -ex \"directory $WORKSPACE_DIR\"
  -ex \"load\"
  -tui
  $elf_file
)
eval "$path_to_arm_none_eabi_gdb ${gdb_args[@]}"

# When gdb is finished, cleanup by stopping openocd
pkill openocd
