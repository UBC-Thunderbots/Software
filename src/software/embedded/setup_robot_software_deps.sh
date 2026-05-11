#!/bin/bash

set -ex

host_software_packages=(
    device-tree-compiler
    curl
    python3
    python3-venv
    python3-pip
    libgpiod-dev
)

# Install packages
sudo apt-get update
sudo apt-get install "${host_software_packages[@]}" -y

# Install platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Why tf do we have a venv? Uh sure here we go
sudo mkdir -p /opt/tbotspython
sudo chown $(whoami):$(whoami) /opt/tbotspython

if ! sudo /usr/local/bin/python3 -m venv /opt/tbotspython ; then
    echo "Error: Installing Python 3 failed"
fi