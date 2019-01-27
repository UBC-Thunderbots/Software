#!/bin/bash
# Start test script for Corner Kick.

export PERCY_TOKEN=369660795fa79afd24f25e21a4f8803e9d56beae94a31fbd4024ad34cf1c8cd1

# Get the current directory of the script.
DIRECTORY=`dirname $0`

# Move to this directory and start the test suite of
# Corner Kick.
cd $DIRECTORY
yarn
yarn test
yarn test:snapshot
