#!/bin/bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Ubuntu Software Setup
#
# This script must be run with sudo! root permissions are required to install
# packages and copy files to the /etc/udev/rules.d directory.
#
# This script will install all the required libraries and dependencies to build
# and run the Thunderbots codebase. This includes being able to run the ai and
# unit tests
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

print_status_msg () {
   echo "================================================================"
   echo $1
   echo "================================================================"
}

add_bashrc_if_not_there () {
    if ! grep -q "$1" ~/.bashrc; then
        echo "$1" >> ~/.bashrc
    fi
}

# Save the parent dir of this so we can always run commands relative to the
# location of this script, no matter where it is called from. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd "$CURR_DIR" || exit

# Get system CPU architecture and Ubuntu version
arch=$(uname -i)
ubuntu_version=$(lsb_release -rs)

# Check that architecture and version are valid
if [[ ("$arch" == "x86_64" && ("$ubuntu_version" == "20.04" || "$ubuntu_version" == "22.04")) || 
      ("$arch" == "aarch64" && "$ubuntu_version" == "22.04") ]]; then
    print_status_msg "Setting up software for architecture $arch and version $ubuntu_version"
else
    print_status_msg "Error: Unsupported environment: architecture $arch and version $ubuntu_version"
    exit 1
fi

# Display warning message for ARM users
if [[ "$arch" == "aarch64" ]]; then
    echo "WARNING: Running this setup script on ARM architecture may cause parts of your Ubuntu installation to become unusable. Continue? [Y/n]"
    read -r answer
    if [[ "$answer" != "" && "$answer" != "y" && "$answer" != "Y" ]]; then
        exit 1
    fi
fi

print_status_msg "Installing Utilities and Dependencies"

sudo apt-get update
sudo apt-get install -y software-properties-common # required for add-apt-repository
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo add-apt-repository -y ppa:deadsnakes/ppa
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
    python3.8       # Python 3
    python3.8-dev # Python 3 headers
    python3.8-venv # Virtual Environment
    python3-pip   # Required for bazel to install python dependencies for build targets
    python3-protobuf # This is required for the "NanoPb" library, which does not
                    # properly manage this as a bazel dependency, so we have
                    # to manually install it ourselves
    python3-yaml 	# Load dynamic parameter configuration files
    valgrind # Checks for memory leaks
    libsqlite3-dev # needed to build Python 3 with sqlite support
    libffi-dev # needed to use _ctypes in Python3
    libssl-dev # needed to build Python 3 with ssl support
    openssl # possibly also necessary for ssl in Python 3
    sshpass #used to remotely ssh into robots via Ansible
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
    host_software_packages+=(qt5-default)
    
    # This fixes missing headers by notifying the linker
    ldconfig
fi

if [[ $(lsb_release -rs) == "22.04" ]]; then
    host_software_packages+=(qtbase5-dev)
fi

if ! sudo apt-get install "${host_software_packages[@]}" -y ; then
    print_status_msg "Error: Installing utilities and dependencies failed"
    exit 1
fi

# Upgrade python3 pip, which some pip packages require
print_status_msg "Setting Up Virtual Python Environment"

# delete tbotspython first
sudo rm -rf /opt/tbotspython

if ! sudo /usr/bin/python3.8 -m venv /opt/tbotspython ; then
    print_status_msg "Error: Setting up virtual environment failed"
    exit 1
fi

if ! sudo /opt/tbotspython/bin/python3 -m pip install --upgrade pip ; then
    print_status_msg "Error: Upgrading pip version in venv failed"
    exit 1
fi

if [[ $(lsb_release -rs) == "20.04" ]]; then
    sudo /opt/tbotspython/bin/pip3 install -r ubuntu20_requirements.txt
fi

if [[ $(lsb_release -rs) == "22.04" ]]; then
    sudo /opt/tbotspython/bin/pip3 install -r ubuntu22_requirements.txt
fi

if [["$arch" == "aarch64"]]; then
    print_status_msg "Starting ARM workarounds for pyqt"
    # There may be a better way to do this that doesn't Frankenstien your ubuntu installation, but this is the only way we found to get pyqt to work.
    # add mantic as source, install python3-pyqt6 and python3-pyqt6.qtwebengine
    sudo echo "deb http://ca.ports.ubuntu.com/ubuntu-ports/ mantic main universe" > /etc/apt/sources.list.d/temp.list
    sudo apt-get update
    sudo apt-get install python3-pyqt6 python3-pyqt6.qtwebengine
    # remove the mantic source
    sudo rm /etc/apt/sources.list.d/temp.list
    # allow tbotspython to access dist-packages
    add_bashrc_if_not_there "export PYTHONPATH=/usr/lib/python3/dist-packages/"
    # fix for pyqtgraph trying to use pyqt5 by default
    add_bashrc_if_not_there "export PYQTGRAPH_QT_LIB=PyQt6"
else
    # else if x86_64, install PyQt6 normally using pip
    sudo /opt/tbotspython/bin/pip3 install -r pyqt6==6.5.0 PyQt6-WebEngine
fi

print_status_msg "Done Setting Up Virtual Python Environment"
print_status_msg "Setting up symlink to qt5 installation"
sudo ln -s /usr/include/$arch-linux-gnu/qt5/ /opt/tbotspython/qt5

print_status_msg "Fetching game controller"

if [[ "$arch" = "aarch64" ]]; then
    GAME_CONTROLLER_LINK=https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v3.7.2/ssl-game-controller_v3.7.2_linux_arm64
else
    GAME_CONTROLLER_LINK=https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v2.15.2/ssl-game-controller_v2.15.2_linux_amd64
fi

sudo chown -R $USER:$USER /opt/tbotspython
sudo wget -nc $GAME_CONTROLLER_LINK -O /opt/tbotspython/gamecontroller
sudo chmod +x /opt/tbotspython/gamecontroller

# Install Bazel
print_status_msg "Installing Bazel"

if [[ "$arch" = "aarch64" ]]; then
    # Download bazelisk, a bazel version manager that has a version for ARM architecture
    sudo wget -nc https://github.com/bazelbuild/bazelisk/releases/download/v1.18.0/bazelisk-linux-arm64 -O /usr/local/bin/bazel
    sudo chmod +x /usr/local/bin/bazel
else
    # Adapted from https://docs.bazel.build/versions/main/install-ubuntu.html#install-with-installer-ubuntu
    sudo wget -nc https://github.com/bazelbuild/bazel/releases/download/5.0.0/bazel-5.0.0-installer-linux-x86_64.sh -O /tmp/bazel-installer.sh
    sudo chmod +x /tmp/bazel-installer.sh
    sudo /tmp/bazel-installer.sh --bin=/usr/bin --base=$HOME/.bazel
fi

# Fix for bazel autocomplete, add it to .bashrc if it isn't there already
add_bashrc_if_not_there "source ${HOME}/.bazel/bin/bazel-complete.bash"

print_status_msg "Done Installing Bazel"
print_status_msg "Setting Up PlatformIO"

# setup platformio to compile arduino code
# link to instructions: https://docs.platformio.org/en/latest/core/installation.html
# **need to reboot for changes to come into effect**

# downloading platformio udev rules
if ! curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/develop/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules; then
    print_status_msg "Error: Downloading PlatformIO udev rules failed"
    exit 1
fi

sudo service udev restart

# allow user access to serial ports
sudo usermod -a -G dialout $USER

# installs PlatformIO to global environment
if ! sudo /usr/bin/python3.8 -m pip install --prefix /usr/local platformio==6.0.2; then
    print_status_msg "Error: Installing PlatformIO failed"
    exit 1
fi

print_status_msg "Done PlatformIO Setup"
print_status_msg "Done Software Setup, please reboot for changes to take place"
