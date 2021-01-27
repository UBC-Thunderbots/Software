#!/bin/bash

# The name of this script
THIS_SCRIPT_FILENAME=$(basename "$0")

# Explicitly allow AI vs AI ports
# Ports found in `constants.h`
sudo ufw allow 42069:42073/tcp
# Enable UFW rules
sudo ufw enable
sudo ufw status
