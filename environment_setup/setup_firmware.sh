#!/usr/bin/env bash

# Setup script to get the arm toolchains for our firmware code.

# Link to the instructions that basically layout how this bash script works:
#   https://gnu-mcu-eclipse.github.io/toolchain/arm/install/

# Directory this script is in
CURR_DIR=$(dirname -- "$(readlink -f -- "$BASH_SOURCE")")

# Install openocd
cd "$CURR_DIR" || exit
./install_openocd.sh

# install cubemx
cd "$CURR_DIR" || exit
./install_cubemx.sh

# Install dfu-util, the tool used to load the firmware onto devices
sudo apt-get update
sudo apt-get install dfu-util -y

# setup platformio to compile arduino code
# link to instructions: https://docs.platformio.org/en/latest/core/installation.html
# **need to reboot for changes to come into effect**

# downloading platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart

# allow user access to serial ports
sudo usermod -a -G dialout $USER

# installs platformio to virtual environment
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"

#sym links executable to directory used by platformio 
sudo ln -sf ~/.platformio/penv/bin/platformio /usr/local/bin/platformio
sudo ln -sf ~/.platformio/penv/bin/pio /usr/local/bin/pio

echo "================================================================"
echo "Done platformio Setup, please reboot for changes to take place"
echo "================================================================"

