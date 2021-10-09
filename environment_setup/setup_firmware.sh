#!/usr/bin/env bash

# setup platformio to compile arduino code
# link to instructions: https://docs.platformio.org/en/latest/core/installation.html
# **need to reboot for changes to come into effect**

# downloading platformio udev rules
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/scripts/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules
sudo service udev restart

# allow user access to serial ports
sudo usermod -a -G dialout $USER

# installs platformio to global environment
sudo pip install --prefix /usr/local platformio==5.1.1
echo "================================================================"
echo "Done platformio Setup, please reboot for changes to take place"
echo "================================================================"
