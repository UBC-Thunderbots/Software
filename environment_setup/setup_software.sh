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

# Save the parent dir of this so we can always run commands relative to the
# location of this script, no matter where it is called from. This
# helps prevent bugs and odd behaviour if this script is run through a symlink
# or from a different directory.
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")
cd "$CURR_DIR" || exit

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

    unzip # installing tigers autoref
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

print_status_msg "Done Setting Up Virtual Python Environment"
print_status_msg "Fetching game controller"

sudo chown -R $USER:$USER /opt/tbotspython
sudo wget -N https://github.com/RoboCup-SSL/ssl-game-controller/releases/download/v2.15.2/ssl-game-controller_v2.15.2_linux_amd64 -O /opt/tbotspython/gamecontroller
sudo chmod +x /opt/tbotspython/gamecontroller

print_status_msg "Setting up TIGERS AutoRef"

print_status_msg "Installing TIGERS dependency: Java 17"
sudo wget -N https://download.oracle.com/java/17/archive/jdk-17.0.5_linux-x64_bin.deb -O /tmp/jdk-17.0.5.deb
sudo apt install /tmp/./jdk-17.0.5.deb

print_status_msg "Compiling TIGERS AutoRef"
sudo wget -N https://github.com/TIGERs-Mannheim/AutoReferee/archive/refs/heads/autoref-ci.zip -O /tmp/autoref-ci.zip
unzip -q -o -d /tmp/ /tmp/autoref-ci.zip
touch /tmp/AutoReferee-autoref-ci/.git # a hacky way to make gradle happy when it tries to find a dependency
/tmp/AutoReferee-autoref-ci/./gradlew installDist -p /tmp/AutoReferee-autoref-ci/ -Dorg.gradle.java.home=/usr/lib/jvm/jdk-17/
cp -r /tmp/AutoReferee-autoref-ci/build/install/autoReferee/ /opt/tbotspython/autoReferee

sudo chmod +x "$CURR_DIR/../src/software/autoref/run_autoref.sh"
sudo cp "$CURR_DIR/../src/software/autoref/DIV_B.txt" "/opt/tbotspython/autoReferee/config/geometry/DIV_B.txt"

print_status_msg "Finished setting up AutoRef"

# Install Bazel
print_status_msg "Installing Bazel"

# Adapted from https://docs.bazel.build/versions/main/install-ubuntu.html#install-with-installer-ubuntu
sudo wget -nc https://github.com/bazelbuild/bazel/releases/download/5.0.0/bazel-5.0.0-installer-linux-x86_64.sh -O /tmp/bazel-installer.sh
sudo chmod +x /tmp/bazel-installer.sh
sudo /tmp/bazel-installer.sh --bin=/usr/bin --base=$HOME/.bazel
echo "source ${HOME}/.bazel/bin/bazel-complete.bash" >> ~/.bashrc

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
