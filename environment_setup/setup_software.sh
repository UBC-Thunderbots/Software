#!/bin/bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Ubuntu Software Setup
#
# This script must be run with sudo! root permissions are required to install
# packages and copy files to the /etc/udev/rules.d directory. The reason that the script
# must be run with sudo rather than the individual commands using sudo, is that
# when running CI within Docker, the sudo command does not exist since
# everything is automatically run as root.
#
# This script will install all the required libraries and dependencies to build
# and run the Thunderbots codebase. This includes being able to run the ai and
# unit tests
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#


# Save the parent dir of this so we can always run commands relative to the
# location of this script, no matter where it is called from. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd "$CURR_DIR" || exit

echo "================================================================"
echo "Installing Utilities and Dependencies"
echo "================================================================"

sudo apt-get update
sudo apt-get install -y software-properties-common # required for add-apt-repository
# Required to make sure we install protobuf version 3.0.0 or greater
sudo add-apt-repository ppa:maarten-fonville/protobuf -y

sudo apt-get update

host_software_packages=(
    cmake # Needed to build some of our dependencies
    codespell # Fixes typos
    curl
    gcc-7 # We use gcc 7.4.0
    kcachegrind # This lets us view the profiles output by callgrind
    libeigen3-dev # A math / numerical library used for things like linear regression
    libprotobuf-dev
    libudev-dev
    libusb-1.0-0-dev
    protobuf-compiler
    protobuf-compiler # This is required for the "NanoPb" library, which does not
                      # properly manage this as a bazel dependency, so we have 
                      # to manually install it ourselves
    python-minimal # This is required for bazel, we've seen some issues where
                   # the bazel install hasn't installed it properly
    python3       # Python 3
    python3-pip   # Required for bazel to install python dependencies for build targets
    python3-protobuf # This is required for the "NanoPb" library, which does not
                    # properly manage this as a bazel dependency, so we have 
                    # to manually install it ourselves
    python3-yaml # yaml for cfg generation (Dynamic Parameters)
    qt5-default # The GUI library for our visualizer
    valgrind # Checks for memory leaks
)

if ! sudo apt-get install "${host_software_packages[@]}" -y ; then
    echo "##############################################################"
    echo "Error: Installing utilities and dependencies failed"
    echo "##############################################################"
    exit 1
fi

echo "================================================================"
echo "Done Installing Newer Valgrind Version"
echo "================================================================"

# Install Bazel
echo "================================================================" 
echo "Installing Bazel"
echo "================================================================"

# Adapted from https://docs.bazel.build/versions/master/install-ubuntu.html#install-on-ubuntu
sudo apt install curl gnupg
curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
sudo apt update
if ! sudo apt install bazel -y ; then
    echo "##############################################################"
    echo "Error: Installing Bazel failed"
    echo "##############################################################"
    exit 1
fi

# Done
echo "================================================================"
echo "Done Software Setup"
echo "================================================================"

