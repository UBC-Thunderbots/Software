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
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt-get update

# (sorted alphabetically)
host_software_packages=(
    cmake # Needed to build some of our dependencies
    codespell # Fixes typos
    curl
    default-jdk # Needed for Bazel to run properly
    gcc-9 # We use gcc 9.3.0
    libstdc++6-9-dbg
    git # required for build
    g++-9
    kcachegrind # This lets us view the profiles output by callgrind
    libeigen3-dev # A math / numerical library used for things like linear regression
    libprotobuf-dev
    libudev-dev
    libusb-1.0-0-dev
    protobuf-compiler # This is required for the "NanoPb" library, which does not
                      # properly manage this as a bazel dependency, so we have
                      # to manually install it ourselves
    python3       # Python 3
    python3-venv # Virtual Environment
    python3-pip   # Required for bazel to install python dependencies for build targets
    python3-protobuf # This is required for the "NanoPb" library, which does not
                    # properly manage this as a bazel dependency, so we have
                    # to manually install it ourselves
    python3-yaml # Load dynamic parameter configuration files
    qt5-default # The GUI library for our visualizer
    tmux        # Used by AI vs AI script
    valgrind # Checks for memory leaks
    libsqlite3-dev # needed to build Python 3 with sqlite support
    libffi-dev # needed to use _ctypes in Python3
    libssl-dev # needed to build Python 3 with ssl support
    openssl # possibly also necessary for ssl in Python 3
)

if [[ $(lsb_release -rs) == "20.04" ]]; then
    # This is required for bazel, we've seen some issues where
    # the bazel install hasn't installed it properly
    host_software_packages+=(python-is-python3)

    # This is to setup the toolchain for bazel to run 
    host_software_packages+=(clang)
    host_software_packages+=(llvm-6.0)
    host_software_packages+=(libclang-6.0-dev)
    host_software_packages+=(libncurses5)
    sudo apt-get -y install gcc-7 g++-7
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-7 7
    
    # This fixes missing headers by notifying the linker
    ldconfig
fi

if [[ $(lsb_release -rs) == "18.04" ]]; then
    # This is required for bazel, we've seen some issues where
    # the bazel install hasn't installed it properly
    host_software_packages+=(python-minimal)
    host_software_packages+=(libclang-dev)
    host_software_packages+=(python2.7-dev)
    host_software_packages+=(python3.8)
    host_software_packages+=(python3.8-dev)
    host_software_packages+=(python3.8-venv)
    host_software_packages+=(python3-setuptools)
fi

if ! sudo apt-get install "${host_software_packages[@]}" -y ; then
    echo "##############################################################"
    echo "Error: Installing utilities and dependencies failed"
    echo "##############################################################"
    exit 1
fi

# Upgrade python3 pip, which some pip packages require
echo "================================================================"
echo "Setting Up Virtual Python Environment"
echo "================================================================"

# delete tbotspython first
sudo rm -rf /opt/tbotspython

if ! sudo /usr/bin/python3.8 -m venv /opt/tbotspython ; then
    echo "##############################################################"
    echo "Error: Setting up virtual environment failed"
    echo "##############################################################"
    exit 1
fi

if ! sudo /opt/tbotspython/bin/python3 -m pip install --upgrade pip ; then
    echo "##############################################################"
    echo "Error: Upgrading pip version in venv failed"
    echo "##############################################################"
    exit 1
fi

if [[ $(lsb_release -rs) == "18.04" ]]; then
    sudo /opt/tbotspython/bin/pip3 install -r ubuntu18_requirements.txt
fi

if [[ $(lsb_release -rs) == "20.04" ]]; then
    sudo /opt/tbotspython/bin/pip3 install -r ubuntu20_requirements.txt
fi


if ! sudo /opt/tbotspython/bin/pip3 install --upgrade protobuf  ; then
    echo "##############################################################"
    echo "Error: Installing protobuf failed"
    echo "##############################################################"
    exit 1
fi

echo "================================================================"
echo "Done Setting Up Virtual Python Environment"
echo "================================================================"

echo "================================================================"
echo "Fetching mobile gamepad and game controller"
echo "================================================================"

FILE=/opt/mobile_gamepad.zip
if [ ! -f "$FILE" ]; then
    echo "Mobile Gamepad Binary not found, fetching"
    # Download handheld controller
    sudo curl -L https://github.com/UBC-Thunderbots/mobile-gamepad/releases/download/vtbots.0.1/gamepad.zip -o /opt/tbotspython/mobile_gamepad.zip
    sudo unzip /opt/mobile_gamepad.zip -d /opt
    sudo chown -R $USER:$USER /opt/mobile_gamepad
fi

sudo wget -nc https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v2.4.0/ssl-game-controller_v2.4.0_linux_amd64 -O /opt/tbotspython/gamecontroller
sudo chmod +x /opt/tbotspython/gamecontroller

# Install Bazel
echo "================================================================"
echo "Installing Bazel"
echo "================================================================"

# Adapted from https://docs.bazel.build/versions/master/install-ubuntu.html#install-on-ubuntu
sudo apt install apt-transport-https curl gnupg -y
curl -fsSL https://bazel.build/bazel-release.pub.gpg | gpg --dearmor > bazel.gpg
sudo mv bazel.gpg /etc/apt/trusted.gpg.d/
echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
sudo apt-get update
if ! sudo apt-get install bazel-5.0.0 -y ; then
    echo "##############################################################"
    echo "Error: Installing Bazel failed"
    echo "If you have a newer version installed, please manually downgrade"
    echo "##############################################################"
    exit 1
fi
sudo rm -f /usr/bin/bazel # remove symlink
sudo ln -s /usr/bin/bazel-5.0.0 /usr/bin/bazel

echo "================================================================"
echo "Done Installing Bazel"
echo "================================================================"

echo "================================================================"
echo "Setting Up PlatformIO"
echo "================================================================"

# setup platformio to compile arduino code
# link to instructions: https://docs.platformio.org/en/latest/core/installation.html
# **need to reboot for changes to come into effect**

# downloading platformio udev rules
if ! curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules; then
    echo "##############################################################"
    echo "Error: Downloading PlatformIO udev rules failed"
    echo "##############################################################"
    exit 1
fi

sudo service udev restart

# allow user access to serial ports
sudo usermod -a -G dialout $USER

# installs PlatformIO to global environment
if ! sudo /usr/bin/python3.8 -m pip install --prefix /usr/local platformio==5.2.4; then
    echo "##############################################################"
    echo "Error: Installing PlatformIO failed"
    echo "##############################################################"
    exit 1
fi


echo "================================================================"
echo "Done PlatformIO Setup"
echo "================================================================"

# Done
echo "================================================================"
echo "Done Software Setup, please reboot for changes to take place"
echo "================================================================"

