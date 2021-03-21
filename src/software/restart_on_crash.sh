#!/bin/bash

# The directory this script is in
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")

# the directory that contains the binary
BIN_DIR=$(pwd)

# The name of this script
THIS_SCRIPT_FILENAME=$(basename "$0")

# The number of arguments passed to this script
NUM_ARGS=$#

if [ $NUM_ARGS -ne 1 ]; then
  echo "Error: Incorrect number of arguments to script"
  echo "Usage: $THIS_SCRIPT_FILENAME [interface_name]"
  echo "Choose from the following interfaces:"
  ls /sys/class/net
  exit 1
fi

if [[ "$1" == "--help" ]]; then
  echo "Runs full system and restarts on any crash"
  echo "Usage: $THIS_SCRIPT_FILENAME [interface_name]"
  echo "Choose from the following interfaces:"
  ls /sys/class/net
  exit 0
fi

cd "$CURR_DIR" || exit
bazel build //software:full_system
cd "$BIN_DIR/../.." || exit

until ./full_system --backend=WifiBackend --interface=$1; do
    echo "Full System crashed with exit code $?.  Respawning.." >&2
    sleep 1
done
