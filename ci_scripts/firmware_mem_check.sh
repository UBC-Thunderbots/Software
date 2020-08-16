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
bazel test --run_under="valgrind --leak-check=yes" //firmware/... > mem_check.out
if [[ "$?" != 0 ]]; then
    echo "There was a problem running the mem check script, stopping now."
    exit 1
fi

cat mem_check.out | grep "LEAK"
if [[ "$?" == 0 ]]; then
    printf "\nMemory leaks detected :( - please run \`bazel test --run_under=\"valgrind --leak-check=yes\" //firmware/...\` to find out why\n\n"
    exit 1
else
    echo "Firmware mem check passed :D"
    exit 0
fi
