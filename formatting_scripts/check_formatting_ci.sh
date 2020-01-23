#!/bin/bash

#
# This script is used for running formatting checks in CI
#

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=========================================================="
echo " THIS SHOULD ONLY BE RUN IN CI, PLEASE DO NOT USE LOCALLY"
echo "=========================================================="

# Run formatting
$CURR_DIR/fix_formatting.sh
if [[ "$?" != 0 ]]; then
    echo "There was a problem running the formatting script, stopping now."
    exit 1
fi

# Check if there are any files changed
GIT_CHANGES=$(git status --porcelain)
if [[ $GIT_CHANGES ]]; then
    # Changes
    echo "FILES CHANGED:"
    echo "$GIT_CHANGES"
    echo "========================================================================"
    echo "formatting check failed :( - please reformat your code via the \`fix_formatting.sh\` script and resubmit"
    exit 1
else
    # No changes
    echo "formatting passed, no files changed :D"
    exit 0
fi

