#!/bin/bash

echo "================================================================"
echo "Installing OpenOCD"
echo "================================================================"

# Any failure should terminate this script
set -e

OPENOCD_TMP_DIR="/tmp/openocd.git"

# Install dependencies required to compile openocd
sudo apt install -y libtool automake

# Setup a clean directory to install openocd in
rm -rf $OPENOCD_TMP_DIR
mkdir -p $OPENOCD_TMP_DIR

# Download openocd
git clone https://github.com/ntfreak/openocd.git $OPENOCD_TMP_DIR

# Build and Install Openocd
cd $OPENOCD_TMP_DIR
sudo cp contrib/60-openocd.rules /etc/udev/rules.d/
sudo adduser $USER plugdev
./bootstrap
./configure
make
sudo make install

echo "================================================================"
echo "Done Installing OpenOCD"
echo "================================================================"
