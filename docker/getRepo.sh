#!/bin/bash

#Script to pull Thunderbots/Software 
#repository from github

echo "Creating Directory"
mkdir Thunderbots || echo "Directory Exists"
cd Thunderbots
git clone https://github.com/UBC-Thunderbots/Software.git || echo "Failed Clone"
# echo "Running setup_software"
# pwd
# cd Software/environment_setup/
# pwd
# ./setup_software.sh || echo "Failed setup_software"