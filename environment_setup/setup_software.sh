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

# Save the parent dir of this so we can always run commands relative to the
# location of this script, no matter where it is called from. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd "$CURR_DIR" || exit

source util.sh

arch=$(uname -m)
print_status_msg "Detected architecture: ${arch}"

print_status_msg "Installing Utilities and Dependencies"

sudo apt-get update
sudo apt-get install -y software-properties-common # required for add-apt-repository
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo add-apt-repository -y ppa:deadsnakes/ppa
sudo apt-get update

# Detect if running under WSL
# See https://github.com/microsoft/WSL/issues/4071#issuecomment-1221588337
if [[ $(grep -i Microsoft /proc/version) ]]; then
    print_status_msg "WSL Setup"

    sudo apt install unzip
    sudo apt install libopengl0 -y

    print_status_msg "Done WSL Setup"
fi

# (sorted alphabetically)
host_software_packages=(
    cmake # Needed to build some of our dependencies
    codespell # Fixes typos
    curl
    default-jdk # Needed for Bazel to run properly
    gcc-10 # Full system compiles with gcc 10
    libstdc++6-9-dbg
    git # required for build
    g++-10
    kcachegrind # This lets us view the profiles output by callgrind
    libeigen3-dev # A math / numerical library used for things like linear regression
    libprotobuf-dev
    libudev-dev
    libusb-1.0-0-dev
    libxcb-cursor0 # This is used as the Linux platform abstraction by Qt 
    protobuf-compiler # This is required for the "NanoPb" library, which does not
                      # properly manage this as a bazel dependency, so we have
                      # to manually install it ourselves
    python3.12        # Python 3
    python3.12-dev    # Python 3 headers
    python3.12-venv   # Virtual Environment
    python3-pip       # Required for bazel to install python dependencies for build targets
    python3-protobuf  # This is required for the "NanoPb" library, which does not
                      # properly manage this as a bazel dependency, so we have
                      # to manually install it ourselves
    python3-yaml 	  # Load dynamic parameter configuration files
    valgrind # Checks for memory leaks
    libsqlite3-dev # needed to build Python 3 with sqlite support
    libffi-dev # needed to use _ctypes in Python3
    libssl-dev # needed to build Python 3 with ssl support
    openssl # possibly also necessary for ssl in Python 3
    sshpass #used to remotely ssh into robots via Ansible
    unzip # installing tigers autoref
    xvfb # used for CI to run GUI applications
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
    
    # This fixes missing headers by notifying the linker
    sudo ldconfig
fi

# Clear the download cache
rm -rf /tmp/tbots_download_cache
mkdir /tmp/tbots_download_cache

if [[ $(lsb_release -rs) == "22.04" ]] || [[ $(lsb_release -rs) == "24.04" ]]; then
    # This is required because a Braille TTY device that Linux provides a driver for conflicts with the ESP32
    wget -nc https://github.com/UBC-Thunderbots/Software-External-Dependencies/blob/main/85-brltty.rules -O /tmp/tbots_download_cache/85-brltty.rules
    sudo mv /tmp/tbots_download_cache/85-brltty.rules /usr/lib/udev/rules.d/85-brltty.rules 
fi

virtualenv_opt_args=""
if [[ $(lsb_release -rs) == "24.04" ]]; then
    host_software_packages+=(python3-pyqt6)
    host_software_packages+=(pyqt6-dev-tools)

    virtualenv_opt_args="--system-site-packages"
fi

if ! sudo apt-get install "${host_software_packages[@]}" -y ; then
    print_status_msg "Error: Installing utilities and dependencies failed"
    exit 1
fi

# Upgrade python3 pip, which some pip packages require
print_status_msg "Setting Up Virtual Python Environment"

# delete tbotspython first
sudo rm -rf /opt/tbotspython

if ! sudo /usr/bin/python3.12 -m venv /opt/tbotspython $virtualenv_opt_args ; then
    print_status_msg "Error: Setting up virtual environment failed"
    exit 1
fi

if [[ $(lsb_release -rs) == "20.04" ]] || [[ $(lsb_release -rs) == "22.04" ]]; then
    # Install pip if it is not a system-managed package
    sudo /usr/bin/python3.12 -m ensurepip
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

if [[ $(lsb_release -rs) == "24.04" ]]; then
    sudo /opt/tbotspython/bin/pip3 install -r ubuntu24_requirements.txt
fi

sudo chown -R $USER:$USER /opt/tbotspython

print_status_msg "Done Setting Up Virtual Python Environment"
print_status_msg "Fetching game controller"
install_gamecontroller $arch

print_status_msg "Setting up TIGERS AutoRef"

print_status_msg "Installing TIGERS dependency: Java 21"
install_java $arch

print_status_msg "Compiling TIGERS AutoRef"
install_autoref $arch

sudo chmod +x "$CURR_DIR/../src/software/autoref/run_autoref.sh"
sudo cp "$CURR_DIR/../src/software/autoref/DIV_B.txt" "/opt/tbotspython/autoReferee/config/geometry/DIV_B.txt"

print_status_msg "Finished setting up AutoRef"

# Install Bazel
print_status_msg "Installing Bazel"

install_bazel $arch

print_status_msg "Done Installing Bazel"

print_status_msg "Install clang-format"
install_clang_format $arch
print_status_msg "Done installing clang-format"

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

# install PlatformIO to global environment
wget -O /tmp/tbots_download_cache/get-platformio.py https://raw.githubusercontent.com/platformio/platformio-core-installer/master/get-platformio.py
if ! /usr/bin/python3.12 /tmp/tbots_download_cache/./get-platformio.py; then
    print_status_msg "Error: Installing PlatformIO failed"
    exit 1
fi

# link platformio to /usr/local/bin so that bazel can find it
sudo rm /usr/local/bin/platformio
sudo ln -s ~/.platformio/penv/bin/platformio /usr/local/bin/platformio

print_status_msg "Done PlatformIO Setup"

print_status_msg "Done Software Setup, please reboot for changes to take place"
