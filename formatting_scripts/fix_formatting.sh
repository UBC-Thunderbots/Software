#!/bin/bash

#
# This script is used for running formatting checks in CI
#

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The root bazel directory
BAZEL_ROOT_DIR="$CURR_DIR/../src"

# Extensions to check formatting for clang-format
CLANG_FORMAT_EXTENSIONS=(h cpp c hpp tpp)

# Function to run clang format
function run_clang_format () {
    echo "Running clang format over all files..."
    cd $BAZEL_ROOT_DIR

    # Generate extension string
    # Formatted as -iname *.EXTENSION -o
    EXTENSION_STRING=""
    for value in "${CLANG_FORMAT_EXTENSIONS[@]}"
    do
        EXTENSION_STRING="$EXTENSION_STRING -iname *.$value -o"
    done

    # Find all the files that we want to format, and pass them to
    # clang-format as arguments
    # We remove the last -o flag from the extension string
    find $CURR_DIR/../src/ ${EXTENSION_STRING::-2} \
        | xargs bazel run @llvm_clang//:clang-format -- -i -style=file
    shift
}

# Function to run bazel formatting
function run_bazel_formatting () {
    echo "Running bazel buildifier to format all bazel files..."
    cd $BAZEL_ROOT_DIR
    bazel run @com_github_bazelbuild_buildtools//:buildifier
}

# Run formatting
run_clang_format
run_bazel_formatting

exit 0
