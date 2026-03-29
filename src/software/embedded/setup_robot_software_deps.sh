#!/bin/bash

set -ex

host_software_packages=(
    device-tree-compiler
    curl
    libssl-dev
    libffi-dev
    zlib1g-dev
    libbz2-dev
    libreadline-dev
    libsqlite3-dev
    libncursesw5-dev
    tk-dev
    libgdbm-dev
    libc6-dev
    liblzma-dev
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
    exit 1
fi

sudo chown -R $USER:$USER /opt/tbotspython

# install PlatformIO to global environment
curl -fsSL -o /tmp/tbots_download_cache/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
if ! /usr/local/bin/python3.12 /tmp/tbots_download_cache/get-platformio.py; then
    echo "Error: Installing PlatformIO failed"
    exit 1
fi

# link platformio to /opt/tbotspython/bin so that bazel can find it
ln -s $HOME/.platformio/penv/bin/platformio /opt/tbotspython/bin/platformio

sudo raspi-config nonint do_serial_hw 0
sudo raspi-config nonint do_serial_cons 1
