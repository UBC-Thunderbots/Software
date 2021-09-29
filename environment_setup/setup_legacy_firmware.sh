#!/usr/bin/env bash

# Setup script to get the arm toolchains for our firmware code.

# Link to the instructions that basically layout how this bash script works:
#   https://gnu-mcu-eclipse.github.io/toolchain/arm/install/

# Directory this script is in
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")

# Install openocd
cd "$CURR_DIR" || exit
./install_openocd.sh

if [[ $? != 0 ]] ; then
    echo "##############################################################"
    echo "Error: Setup Firmware installing OpenOCD failed"
    echo "##############################################################"
    exit 1
fi

# install cubemx
cd "$CURR_DIR" || exit
./install_cubemx.sh

if [[ $? != 0 ]] ; then
    echo "##############################################################"
    echo "Error: Setup Firmware installing CubeMX failed"
    echo "##############################################################"
    exit 1
fi

# Install dfu-util, the tool used to load the firmware onto devices
sudo apt-get update
sudo apt-get install dfu-util -y
