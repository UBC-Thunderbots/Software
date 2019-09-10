#!/bin/bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Linux grSim Setup
#
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
    build-essential
    cmake
    libqt4-dev
    libgl1-mesa-dev
    libglu1-mesa-dev
    libprotobuf-dev
    protobuf-compiler
    libode-dev
    libboost-dev
)

# Update the list of sources
sudo apt-get update

# Install the packages
sudo apt-get install ${host_software_packages[@]} -y
if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Installing grsim dependencies failed"
    echo "##############################################################"
    exit 1
fi


# Install vartypes. These are required by grSim
echo "================================================================"
echo "Installing Vartypes"
echo "================================================================"

# Clone and install vartypes.
# Instructions taken from the grsim installation guide
# https://github.com/RoboCup-SSL/grSim/blob/master/INSTALL.md

# Remove old vartypes if it exists
vartypes_location="/tmp"
vartypes_path="$vartypes_location/vartypes"

if [ -d $vartypes_path ]; then
    echo "Removing old vartypes..."
    sudo rm -rf $vartypes_path
fi

cd $vartypes_location
git clone https://github.com/jpfeltracco/vartypes.git 
cd vartypes
mkdir build
cd build
cmake ..
make
sudo make install

# Return to the parent directory of the script so commands are relative to here
cd $CURR_DIR


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
sudo ln -sf $grSim_path/bin/grsim /usr/local/bin/grsim


# Report done
echo "================================================================"
echo "Done grSim Setup"
echo "================================================================"

