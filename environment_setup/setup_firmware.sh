#!/usr/bin/env bash

# Setup script to get the arm toolchains for our firmware code.

# Link to the instructions that basically layout how this bash script works:
#   https://gnu-mcu-eclipse.github.io/toolchain/arm/install/

# Temporary setup folder to download and unpack tarball
TEMP_FOLDER="/tmp/temp-thunderbots-gcc-setup"

# Output name for tarball
OUTPUT_TAR_NAME="gcc-arm-none-eabi-7.tar.bz2"

# The link to the arm-none-eabi tarball
# got the link here: https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads
GCC_LINK="https://developer.arm.com/-/media/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2?revision=375265d4-e9b5-41c8-bf23-56cbe927e156?product=GNU%20Arm%20Embedded%20Toolchain,64-bit,,Linux,7-2017-q4-major"

# The folder name of the unpacked tarball
UNPACKED_TAR="gcc-arm-none-eabi-7-2017-q4-major"

# Location to store the toolchains
TBOTS_TOOLCHAIN="/opt/tbots-arm-toolchain/"

# Create a temporary setup folder
# Delete it if it already exists to we have a fresh install
if [ -d $TEMP_FOLDER ]; then
    sudo rm -r $TEMP_FOLDER
fi
mkdir -p $TEMP_FOLDER

# Create the folder to store the toolchain
# Since /opt is owned by the system this requires sudo
# We delete the folder if it already exists so we get a fresh install
if [ -d $TBOTS_TOOLCHAIN ]; then
    sudo rm -r $TBOTS_TOOLCHAIN
fi
sudo mkdir -p $TBOTS_TOOLCHAIN

# go into the temporary setup folder
cd $TEMP_FOLDER

# download the arm-none-eabi
wget -O $OUTPUT_TAR_NAME $GCC_LINK
if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Downloading arm toolchain failed"
    echo "##############################################################"
    exit 1
fi
# unpack it
tar xjf $OUTPUT_TAR_NAME
# change permissions
chmod -R -w $UNPACKED_TAR

# go into unpacked arm-none-eabi
cd $UNPACKED_TAR
# move all of the folders to our tbots toolchain location
sudo mv ./* $TBOTS_TOOLCHAIN

# remove the temporary setup folder
sudo rm -r $TEMP_FOLDER

# Install dfu-util, the tool used to load the firmware onto devices
sudo apt-get update
sudo apt-get install dfu-util -y

