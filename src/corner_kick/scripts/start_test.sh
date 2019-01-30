#!/bin/bash
# Start test script for Corner Kick.

# Get the current directory of the script.
DIRECTORY=`dirname $0`

# Move to this directory and start the test suite of
# Corner Kick.
cd $DIRECTORY
yarn test:coverage
