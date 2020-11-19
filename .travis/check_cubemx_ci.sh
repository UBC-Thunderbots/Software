#!/bin/bash

#
# This script is used to check for corrupt ioc files in CI
#

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=========================================================="
echo " THIS SHOULD ONLY BE RUN IN CI, PLEASE DO NOT USE LOCALLY"
echo "=========================================================="

cd src
bazel run --cpu=stm32h7 //firmware_new/tools:cubemx_regen firmware_new/boards/frankie_v1
if [[ "$?" != 0 ]]; then
    echo "There was an regenerating cubemx, stopping now."
    exit 1
fi

cd ..
# Fix formatting
./formatting_scripts/fix_formatting.sh
git diff --exit-code
if [[ "$?" != 0 ]]; then
    echo "Code generated is different from code committed, stopping now."
    exit 1
fi
