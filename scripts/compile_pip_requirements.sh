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
bazel run //extlibs:nanopb_requirements.update
bazel run //software/thunderscope:requirements.update
bazel run //software/embedded/ansible:requirements.update
bazel run //software/simulated_tests:requirements.update
