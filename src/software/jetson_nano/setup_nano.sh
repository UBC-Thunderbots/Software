#!/bin/bash


# (sorted alphabetically)
host_software_packages=(
    libjpeg8-dev
    libjpeg-dev
    python3.8       # Python 3
    python3.8-venv # Virtual Environment
    zlib1g-dev
)

sudo apt-get update

sudo add-apt-repository -y ppa:deadsnakes/ppa

if ! sudo apt-get install "${host_software_packages[@]}" -y ; then
    echo "##############################################################"
    echo "Error: Installing utilities and dependencies failed"
    echo "##############################################################"
    exit 1
fi

if ! sudo /usr/bin/python3.8 -m venv /opt/tbotspython ; then
    echo "##############################################################"
    echo "Error: Setting up virtual environment failed"
    echo "##############################################################"
    exit 1
fi

echo "##############################################################"
echo "Finished Nano Setup Script"
echo "##############################################################"

exit 0
