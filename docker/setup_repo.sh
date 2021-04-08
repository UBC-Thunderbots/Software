#!/bin/bash

# Script to pull Thunderbots/Software 
# repository from github and run
# setup_software script

echo "Creating Directory"
mkdir Thunderbots || echo "Directory Exists"
cd Thunderbots
git clone https://github.com/UBC-Thunderbots/Software.git || echo "Failed Clone"
cd Software/environment_setup
./setup_software.sh