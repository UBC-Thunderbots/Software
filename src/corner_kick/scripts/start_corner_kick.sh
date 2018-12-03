#!/bin/bash
# Start script for Corner Kick.

# Get the current directory of the script.
DIRECTORY=`dirname $0`

# Move to this directory and start the production build of
# Corner Kick.
cd $DIRECTORY
yarn prod
