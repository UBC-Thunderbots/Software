#!/bin/bash

#
# This script is used for checking for memory leaks in firmware tests
#

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=========================================================="
echo " THIS SHOULD ONLY BE RUN IN CI, PLEASE DO NOT USE LOCALLY"
echo "=========================================================="

# Run formatting
cd $CURR_DIR/../src
bazel test --run_under="valgrind --leak-check=yes --error-exitcode=1 --undef-value-errors=no" //firmware/...
if [[ "$?" != 0 ]]; then
    printf "\nMemory leaks detected :( - please run \`bazel test --run_under=\"valgrind --leak-check=yes\" //firmware/...\` to find out why\n\n"
    exit 1
else
    printf "\nFirmware mem check passed :D\n\n"
    exit 0
fi
