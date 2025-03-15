#!/bin/bash

set -ex

host_software_packages=(
    redis
    device-tree-compiler
    curl
)

# Install packages
sudo apt-get update
sudo apt-get install "${host_software_packages[@]}" -y

# Install platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Install Python 3.12
mkdir -p /tmp/tbots_download_cache
wget -N https://www.python.org/ftp/python/3.12.0/Python-3.12.0.tar.xz -O /tmp/tbots_download_cache/python-3.12.0.tar.xz
tar -xf /tmp/tbots_download_cache/python-3.12.0.tar.xz -C /tmp/tbots_download_cache/
cd /tmp/tbots_download_cache/Python-3.12.0
./configure --enable-optimizations
make -j$(nproc)
sudo make altinstall

if ! sudo /usr/local/bin/python3.12 -m venv /opt/tbotspython ; then
    echo "Error: Installing Python 3.12 failed"
fi
