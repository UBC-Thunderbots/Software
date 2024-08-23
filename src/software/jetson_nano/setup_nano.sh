#!/bin/bash

set -ex

host_software_packages=(
    redis
    libjpeg8-dev
    libjpeg-dev
    device-tree-compiler
    zlib1g-dev
    curl
)

sudo apt-get update
sudo add-apt-repository -y ppa:deadsnakes/ppa

# Install packages
sudo apt-get install "${host_software_packages[@]}" -y

# Install platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
