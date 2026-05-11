#!/bin/bash

set -ex

host_software_packages=(
    git
    autoconf
    libtool
    make
    pkg-config
    libusb-1.0-0
    libusb-1.0-0-dev
)

# Install packages
sudo apt-get update
sudo apt-get install "${host_software_packages[@]}" -y

# OpenOCD for MD fLash Installation
if ! command -v openocd &> /dev/null; then
    cd ~
    # Using GitHub mirror because zylin Gerrit can be slow/unstable
    git clone --depth 1 https://github.com/openocd-org/openocd.git
    cd openocd
    ./bootstrap
    ./configure --enable-sysfsgpio --enable-linuxgpiod
    make -j$(nproc)
    sudo make install
fi
