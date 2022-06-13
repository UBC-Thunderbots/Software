#!/bin/bash

set -ex

host_software_packages=(
    redis
    libjpeg8-dev
    libjpeg-dev
    python3.8      # Python 3
    python3.8-venv # Virtual Environment
    python3.8-dev  # Python 3 Headers
    zlib1g-dev
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
    Adafruit-Blinka==6.15.0
    adafruit-circuitpython-busdevice==5.1.1
    adafruit-circuitpython-rgb-display==3.10.9
    Adafruit-PlatformDetect==3.18.0
    Adafruit-PureIO==1.1.9
    Jetson.GPIO==2.0.17
    Pillow==9.1.0
    redis==4.1.4
)

# Install python dependencies
sudo /opt/tbotspython/bin/pip3.8 install "${pip_libaries[@]}"

