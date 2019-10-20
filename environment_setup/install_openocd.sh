#!/bin/bash

echo "================================================================"
echo "Installing OpenOCD"
echo "================================================================"

# Any failure should terminate this script
set -e

OPENOCD_TMP_DIR="/tmp/openocd.git"

# Setup a clean directory to install openocd in
rm -rf $OPENOCD_TMP_DIR
mkdir -p $OPENOCD_TMP_DIR

# Download openocd
git clone https://github.com/ntfreak/openocd.git $OPENOCD_TMP_DIR

# Build and Install Openocd
cd $OPENOCD_TMP_DIR
./bootstrap
./configure
make
sudo make install

echo "================================================================"
echo "Done Installing OpenOCD"
echo "================================================================"
