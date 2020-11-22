#!/bin/bash

# The name of this script
THIS_SCRIPT_FILENAME=$(basename "$0")

# The number of arguments passed to this script
NUM_ARGS=$#

if [ $NUM_ARGS -ne 1 ]; then
  echo "Error: Incorrect number of arguments to script"
  echo "Usage: $THIS_SCRIPT_FILENAME [interface_name]"
  echo "Choose from the following interfaces:"
  echo "`ls /sys/class/net`"
  exit 1
fi

if [[ "$1" == "help" ]]; then
  echo "Runs AI vs AI with the standalone simulator"
  echo "Usage: $THIS_SCRIPT_FILENAME [interface_name]"
  echo "Choose from the following interfaces:"
  echo "`ls /sys/class/net`"
  exit 0
fi

bazel run //software:full_system -- --backend=WifiBackend --interface=$1 &
bazel run //software:full_system -- --backend=WifiBackend --interface=$1 &
bazel run //software/simulation:standalone_simulator_main -- --interface=$1 &
wget -nc https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v2.4.0/ssl-game-controller_v2.4.0_linux_amd64 -O /tmp/ssl-game-controller_v2.4.0_linux_amd64
chmod +x /tmp/ssl-game-controller_v2.4.0_linux_amd64
/tmp/ssl-game-controller_v2.4.0_linux_amd64 &
xdg-open http://localhost:8081 &
