#!/bin/bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# This script runs all the `compile_pip_requirements` update targets in the repo
# to generate requirements lock files from requirement.in files. 
#
# Whenever we update requirements.in with new dependencies/versions, we need to 
# regenerate its associated requirements lock file and check it into git.
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The root bazel directory
BAZEL_ROOT_DIR="$CURR_DIR/../src"

cd $BAZEL_ROOT_DIR
bazel run //software/thunderscope:requirements.update
bazel run //software/embedded/ansible:requirements.update
bazel run //software/gameplay_tests:requirements.update
bazel run //software/embedded/robot_diagnostics_cli:requirements.update
bazel run //starlark/nanopb:requirements.update

# Restore any lock file a failed update left empty so we don't commit a deleted lock file,
# then exit non-zero so we know to re-run the failed script
exit_code=0
for lock in $(git ls-files '*requirements_lock*.txt'); do
    if [ ! -s "$lock" ]; then
        git restore "$lock"
        echo "Restored $lock: its requirements.update target failed, please re-run"
        exit_code=1
    fi
done
exit $exit_code
