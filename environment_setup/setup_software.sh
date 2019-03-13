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
# This is where we want to add aliases, source ROS environment
# variables, etc.
SHELL_CONFIG_FILES=(
    "$HOME/.bashrc"\
    "$HOME/.zshrc"
)

# All lines listed here will be added to the shell config files
# listed above, if they are not present already
# These automatically perform functions outlined in the ROS setup tutorial
# http://wiki.ros.org/melodic/Installation/Ubuntu
declare -a new_shell_config_lines=(
    # Source the ROS Environment Variables Automatically
    "source /opt/ros/melodic/setup.sh"\
    # Source the setup script for our workspace. This normally needs to
    # be done manually each session before you can work on the workspace,
    # so we put it here for convenience.
    "source $GIT_ROOT/devel/setup.sh"\
    # Aliases to make development easier
    # You need to source the setup.sh script before launching CLion so it can
    # find catkin packages
    "alias clion=\"source /opt/ros/melodic/setup.sh \
        && source $GIT_ROOT/devel/setup.sh \
        && clion & disown \
        && exit\""\
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

# Install ROS
echo "================================================================" 
echo "Installing ROS Melodic"
echo "================================================================"

# See http://wiki.ros.org/melodic/Installation/Ubuntu for instructions
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-melodic-desktop -y
if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Installing ROS failed"
    echo "##############################################################"
    exit 1
fi
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential


echo "================================================================"
echo "Installing other ROS dependencies specified by our packages"
echo "================================================================"

# Update Rosdeps
# rosdep init only needs to be called once after installation.
# Check if the sources.list already exists to determing if rosdep has
# been installed already
if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]
then
    sudo rosdep init
fi

rosdep update
# Install all required dependencies to build this repo
rosdep install --from-paths $CURR_DIR/../src --ignore-src --rosdistro melodic -y
if [ $? -ne 0 ]; then
    echo "##############################################################"
    echo "Error: Installing ROS dependencies failed"
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
    python-rosinstall
    protobuf-compiler
    libprotobuf-dev
    libsigc++-2.0-dev
    libusb-1.0-0-dev
    nodejs # Installed directly instead of using rosdep due to the lack of a default PPA
    yarn # Installed directly instead of using rosdep due to the lack of a default PPA 
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

