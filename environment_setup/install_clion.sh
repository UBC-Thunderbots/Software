#!/bin/bash

echo "================================================================"
echo "Installing CLion"
echo "================================================================"

# Clion can be installed through snapcraft
# https://snapcraft.io/clion
sudo apt-get install snapcraft
sudo snap install clion --classic

echo "================================================================"
echo "Done"
echo "================================================================"

