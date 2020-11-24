#!/bin/bash

#
# This script is used to check for corrupt ioc files in CI
#

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=========================================================="
echo " THIS SHOULD ONLY BE RUN IN CI, PLEASE DO NOT USE LOCALLY"
echo "=========================================================="

# In order to run cubemx in CI, we need to have an X-Server running
# _even though_ the process is completely scripted and does not need the GUI at all
# So we install a virutal Xserver to make cubemx happy
sudo apt-get install xvfb
Xvfb -shmem -screen 0 1280x1024x24

cd src
bazel run --cpu=stm32h7 //firmware_new/tools:cubemx_regen firmware_new/boards/frankie_v1
if [[ "$?" != 0 ]]; then
    echo "There was an regenerating cubemx, stopping now."
    exit 1
fi

cd ..

# Fix formatting
git diff --ignore-space-at-eol -b -w --ignore-blank-lines --exit-code
if [[ "$?" != 0 ]]; then
    echo "Code generated is different from code committed, stopping now."
    exit 1
fi
