#!/bin/bash

set -ex

host_software_packages=(
    redis
    device-tree-compiler
    python3.8      # Python 3
    python3.8-venv # Virtual Environment
    python3.8-dev  # Python 3 Headers
    curl
)

sudo apt-get update
sudo add-apt-repository -y ppa:deadsnakes/ppa

# Install packages
sudo apt-get install "${host_software_packages[@]}" -y

# Delete tbotspython first
sudo rm -rf /opt/tbotspython

# Setup python3.8 venv
sudo /usr/bin/python3.8 -m venv /opt/tbotspython
sudo /opt/tbotspython/bin/python3 -m pip install --upgrade pip

pip_libaries=(
    setuptools==60.5.0
    redis==4.1.4
    platformio==6.0.2
)

# Install python dependencies
sudo /opt/tbotspython/bin/pip3.8 install "${pip_libaries[@]}"

# Install platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
