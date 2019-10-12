#!/bin/bash

#########################################################################################
# WARNING: THIS SCRIPT IS ONLY FOR ARCH LINUX SYSTEMS, AND IS NOT OFFICIALLY SUPPORTED. #
# IT IS NOT RUN IN CI AND HAS NO GUARANTEE OF WORKING!                                  #
# (but you are free to fix it if it doesn't :) )                                        #
######################################################################################### 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Arch Linux Software Setup
#
# This script will install all the required libraries and dependencies to build
# and run the Thunderbots codebase. This includes being able to run the ai and
# unit tests
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#


# Save the parent dir of this so we can always run commands relative to the
# location of this script, no matter where it is called from. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd $CURR_DIR
# The root directory of the reopsitory and git tree
GIT_ROOT="$CURR_DIR/.."


echo "================================================================"
echo "Installing Utilities and Dependencies"
echo "================================================================"

host_software_packages=(
    bazel
    curl
    cmake
    protobuf
    libusb
    qt5-base # The GUI library for our visualizer
    eigen # A math / numerical library used for things like linear regression
    python-yaml # yaml for cfg generation (Dynamic Parameters)
)
sudo pacman -Sy
sudo pacman -S ${host_software_packages[@]} --noconfirm

if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Installing utilities and dependencies failed"
    echo "##############################################################"
    exit 1
fi

# Symlink qt include directory
# As the Qt Bazel rules depend on an include directory which varies between Linux
# platforms, we symlink this directory to one place in our source tree
# and platform-dependent properties stay in the setup script
ln -snf /usr/include/qt $GIT_ROOT/src/external/qt

# Done
echo "================================================================"
echo "Done Software Setup"
echo "================================================================"
