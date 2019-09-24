#!/bin/bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# UBC Thunderbots Linux Software Setup
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
cd $CURR_DIR
# The root directory of the reopsitory and git tree
GIT_ROOT="$CURR_DIR/.."


echo "================================================================"
echo "Setting up your shell config files"
echo "================================================================"
# Shell config files that various shells source when they run.
# This is where we want to add aliases, etc.
SHELL_CONFIG_FILES=(
    "$HOME/.bashrc"\
    "$HOME/.zshrc"
)

# All lines listed here will be added to the shell config files
# listed above, if they are not present already
declare -a new_shell_config_lines=(
    "alias rviz=\"rviz & disown && exit\""\
    "alias rqt=\"rqt & disown && exit\""
)

# Add all of our new shell config options to all the shell
# config files, but only if they don't already have them
for file_name in "${SHELL_CONFIG_FILES[@]}";
do
    if [ -f "$file_name" ]
    then
        echo "Setting up $file_name"
        for line in "${new_shell_config_lines[@]}";
        do
            if ! grep -Fq "$line" $file_name
            then
                echo "$line" >> $file_name
            fi
        done
    fi
done

# Install Bazel
echo "================================================================" 
echo "Installing Bazel"
echo "================================================================"

# Adapted from https://docs.bazel.build/versions/master/install-ubuntu.html#install-on-ubuntu
sudo apt-get update
sudo apt-get install openjdk-11-jdk -y
echo "deb [arch=amd64] https://storage.googleapis.com/bazel-apt stable jdk1.8" | sudo tee /etc/apt/sources.list.d/bazel.list
curl https://bazel.build/bazel-release.pub.gpg | sudo apt-key add -
sudo apt-get update
sudo apt-get install bazel -y
if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Installing Bazel failed"
    echo "##############################################################"
    exit 1
fi

echo "================================================================"
echo "Installing Misc. Utilities"
echo "================================================================"

sudo apt-get update
sudo apt-get install -y software-properties-common # required for add-apt-repository
# Required to make sure we install protobuf version 3.0.0 or greater
sudo add-apt-repository ppa:maarten-fonville/protobuf -y

# Running a PPA setup script to give us access to the correct node and yarn version
# on non-Ubuntu 18.04 systems. 
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | sudo apt-key add -
echo "deb https://dl.yarnpkg.com/debian/ stable main" | sudo tee /etc/apt/sources.list.d/yarn.list

sudo apt-get update

host_software_packages=(
    g++-7 # We need g++ 7 or greater to support the C++17 standard
    protobuf-compiler
    libprotobuf-dev
    libusb-1.0-0-dev
    qt5-default # The GUI library for our visualizer
    libudev-dev
    libeigen3-dev # A math / numerical library used for things like linear regression
    yarn
)
sudo apt-get install ${host_software_packages[@]} -y

if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Installing utilities failed"
    echo "##############################################################"
    exit 1
fi

sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-7 
sudo update-alternatives --config gcc

# Clone, build, and install g3log. Adapted from instructions at:
# https://github.com/KjellKod/g3log
g3log_path="/tmp/g3log"
if [ -d $g3log_path ]; then
    echo "Removing old g3log..."
    sudo rm -r $g3log_path
fi

git clone https://github.com/KjellKod/g3log.git $g3log_path
cd $g3log_path
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
cd $CURR_DIR

# Clone, build, and install munkres-cpp (Our Hungarian library algorithm)
hungarian_path="/tmp/hungarian-cpp"
if [ -d $hungarian_path ]; then
    echo "Removing old hungarian-cpp library..."
    sudo rm -r $hungarian_path
fi

git clone https://github.com/saebyn/munkres-cpp.git $hungarian_path
cd $hungarian_path
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make
sudo make install
cd $CURR_DIR

# yaml for cfg generation (Dynamic Parameters)
sudo apt-get install python3-yaml -y

# Done
echo "================================================================"
echo "Done Software Setup"
echo "================================================================"

