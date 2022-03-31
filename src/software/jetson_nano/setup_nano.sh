#!/bin/bash
sudo apt-get update

# (sorted alphabetically)
host_software_packages=(
    python3       # Python 3
    python3-venv # Virtual Environment
)

if ! sudo apt-get install "${host_software_packages[@]}" -y ; then
    echo "##############################################################"
    echo "Error: Installing utilities and dependencies failed"
    echo "##############################################################"
    exit 1
fi

if ! sudo /usr/bin/python3.6 -m venv /opt/tbotspython ; then
    echo "##############################################################"
    echo "Error: Setting up virtual environment failed"
    echo "##############################################################"
    exit 1
fi