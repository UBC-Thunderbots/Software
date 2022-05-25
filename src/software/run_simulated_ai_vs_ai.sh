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

# download SSL GameController if not downloaded already
wget -nc https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v2.4.0/ssl-game-controller_v2.4.0_linux_amd64 -O /tmp/ssl-game-controller_v2.4.0_linux_amd64
chmod +x /tmp/ssl-game-controller_v2.4.0_linux_amd64

tmux new-session \; \
  set -g mouse on \; \
  send-keys "bazel run //software:full_system -- --backend=SimulatorBackend --channel=0 --team_color=yellow --defending_side=negative --interface=$1" C-m \; \
  split-window -h \; \
  send-keys "bazel run //software:full_system -- --backend=SimulatorBackend --channel=1 --team_color=blue --defending_side=positive --interface=$1" C-m \; \
  split-window -v \; \
  send-keys "/tmp/ssl-game-controller_v2.4.0_linux_amd64&" C-m \; \
  send-keys "xdg-open http://localhost:8081" C-m \; \
  send-keys "fg" C-m \; \
  select-pane -t 0 \; \
  split-window -v \; \
  send-keys "bazel run //software:standalone_simulator_main -- --interface=$1" C-m \;
