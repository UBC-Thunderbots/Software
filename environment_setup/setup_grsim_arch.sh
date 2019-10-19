#!/bin/bash

#########################################################################################
# WARNING: THIS SCRIPT IS ONLY FOR ARCH LINUX SYSTEMS, AND IS NOT OFFICIALLY SUPPORTED. #
# IT IS HAS NO GUARANTEE OF WORKING!                                                    #
# (but you are free to fix it if it doesn't :) )                                        #
######################################################################################### 
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Arch Linux grSim Setup
# This script will install all the required libraries and dependencies to run
# grSim. This only includes the simulator (grSim), and not anything related to
# our AI. See the setup_software.sh script for that.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# Save the parent dir of this script since we want to return here
# so that commands are run relative to the location of this script. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd $CURR_DIR


# Install required packages
echo "================================================================"
echo "Downloading and installing required packages."
echo "================================================================"

# The list of packages required to build and run grSim
# See the github repo for the list of dependencies
# https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md

host_software_packages=(
    git
    cmake
    protobuf
    qt5-base
    protobuf
    ode
    boost
)

sudo pacman -Sy
sudo pacman -S ${host_software_packages[@]} --noconfirm

if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Installing grsim dependencies failed"
    echo "##############################################################"
    exit 1
fi

# Install grSim
echo "================================================================"
echo "Installing grSim"
echo "================================================================"

# Clone and install grSim.
# grsim will be install in the /opt directory
# Since /opt is owned by the system commands that operate on it
# (adding and removing files) require sudo

# Remove old grsim if it exists
grSim_location="/opt"
grSim_path="$grSim_location/grSim"

if [ -d $grSim_path ]; then
    echo "Removing old grSim..."
    sudo rm -r $grSim_path
fi

cd $grSim_location
sudo git clone https://github.com/RoboCup-SSL/grSim
cd ./grSim
sudo mkdir build
cd build
sudo cmake ..
sudo make

# Create a symlink so that the grsim executable will be in the PATH
# /usr/local/bin is one of the default locations included by the PATH
sudo ln -sf $grSim_path/bin/grSim /usr/local/bin/grsim


# Report done
echo "================================================================"
echo "Done grSim Setup"
echo "================================================================"

