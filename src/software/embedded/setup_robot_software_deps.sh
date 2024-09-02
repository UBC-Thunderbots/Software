#!/bin/bash

set -ex

host_software_packages=(
    redis
    device-tree-compiler
    curl
    # add-apt-repository is unavailable on Debian platforms, so we install a consistent python from source. At a later
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

# Install platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
