#!/bin/bash

set -ex

host_software_packages=(
    redis
    device-tree-compiler
    curl
    # add-apt-repository is unavailable on Debian platforms, so we install a consitent python from source. At a later
    # time, consider upgrading python
    build-essential
    gdb
    lcov
    pkg-config
    libbz2-dev
    libffi-dev
    libgdbm-dev
    libgdbm-compat-dev
    liblzma-dev
    libncurses5-dev
    libreadline6-dev
    libsqlite3-dev
    libssl-dev
    lzma
    lzma-dev
    tk-dev
    uuid-dev
    zlib1g-dev
)

# Install packages
sudo apt-get update
sudo apt-get install "${host_software_packages[@]}" -y

# Install python3.8 from source
sudo wget -O /tmp/python.tar.xz https://www.python.org/ftp/python/3.8.19/Python-3.8.19.tar.xz
tar -xf /tmp/python.tar.xz -C /tmp
cd /tmp/Python-3.8.19
sudo ./configure --enable-optimizations
sudo make altinstall > /tmp/python_install.log
cd -

# Delete tbotspython first
sudo rm -rf /opt/tbotspython

# Setup python3.8 venv
sudo python3.8 -m venv /opt/tbotspython
sudo /opt/tbotspython/bin/python3 -m pip install --upgrade pip

pip_libaries=(
    setuptools==60.5.0
    redis==4.1.4
    platformio==6.0.2
)

# Install python dependencies
sudo /opt/tbotspython/bin/pip3.8 install "${pip_libaries[@]}" --default-timeout=100

# Install platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
