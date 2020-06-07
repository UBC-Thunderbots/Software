#!/bin/bash

# Save the parent dir of this script since we want to return here
# so that commands are run relative to the location of this script. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd "$CURR_DIR" || exit

# Set up udev rules for USB permissions. This gives the user the required permissions
# to use the radio dongle and flash the robots via USB.
echo "================================================================"
echo "Setting up Thunderbots udev rules"
echo "================================================================"
# Target file directory
udev_rules_file="/etc/udev/rules.d/99-thunderbots.rules"

# Copy the new Thunderbots udev rules to the rule folder, overwriting any
# existing files

if ! sudo cp "$CURR_DIR/99-thunderbots.rules" "$udev_rules_file" ; then
    echo "##############################################################"
    echo "Error: Failed to copy udev rules"
    echo "##############################################################"
    exit 1
fi

# Add user to dialout group for USB comms
sudo adduser "$USER" dialout

# Create a new 'thunderbots' group that is used by the udev file
# This allows the udev rules file to properly grant USB permission
sudo groupadd thunderbots
sudo adduser "$USER" thunderbots

echo "================================================================"
echo "Done"
echo ""
echo "Make sure to logout or restart your computer for permissions changes to take effect!"
echo "================================================================"

