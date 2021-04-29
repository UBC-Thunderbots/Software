#!/bin/bash

echo "================================================================"
echo "Installing OpenOCD"
echo "================================================================"

# Any failure should terminate this script
set -e

OPENOCD_TMP_DIR="/tmp/openocd.git"

# Install dependencies required to compile openocd
sudo apt-get install -y libtool automake

# Setup a clean directory to install openocd in
rm -rf $OPENOCD_TMP_DIR
mkdir -p $OPENOCD_TMP_DIR

# Download openocd
git clone https://github.com/ntfreak/openocd.git $OPENOCD_TMP_DIR
cd $OPENOCD_TMP_DIR

# Set permissions to allow this user to access USB devices
sudo cp contrib/60-openocd.rules /etc/udev/rules.d/
sudo adduser "$USER" plugdev

# Build and Install Openocd
./bootstrap
./configure
make
sudo make install

echo "================================================================"
echo "Done Installing OpenOCD"
echo ""
echo "Make sure to logout or restart your computer for permissions changes to take effect!"
echo "================================================================"
