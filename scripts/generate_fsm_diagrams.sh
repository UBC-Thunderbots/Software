#!/bin/bash

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#
# This script generates the FSM diagrams using Bazel's hermetic Python
#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The root bazel directory
BAZEL_ROOT_DIR="$CURR_DIR/../src"

printf "Generating FSM diagrams...\n\n"

# `bazel run` gives the generator the paths it needs to write
# docs/fsm-diagrams.md at the repository root
(cd "$BAZEL_ROOT_DIR" && bazel run //starlark:fsm_diagram_generator)

if [[ "$?" != 0 ]]; then
    printf "\n***Failed to generate FSM diagrams!***\n\n"
    exit 1
fi
