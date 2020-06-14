#!/bin/bash

#
# This script is used for running formatting checks in CI
#

# The version of the clang executable to use
export CLANG_VERSION=7.0
# The version of black to use
export BLACK_VERSION=19-10b0

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The root bazel directory
BAZEL_ROOT_DIR="$CURR_DIR/../src"

# Extensions to check formatting for clang-format
CLANG_FORMAT_EXTENSIONS=(h cpp c hpp tpp proto)

# Function to run clang format
function run_clang_format () {
    echo "Running clang format over all files..."
    cd $BAZEL_ROOT_DIR

    # Generate extension string
    # Formatted as -iname *.EXTENSION -o
    EXTENSION_STRING=""
    for value in "${CLANG_FORMAT_EXTENSIONS[@]}"
    do
        EXTENSION_STRING="$EXTENSION_STRING -name *.$value -o"
    done

    # Find all the files that we want to format, and pass them to
    # clang-format as arguments
    # We remove the last -o flag from the extension string
    find $CURR_DIR/../src/ ${EXTENSION_STRING::-2}  \
        | xargs -I{} -n1000 $CURR_DIR/clang-format-$CLANG_VERSION -i -style=file

    if [[ "$?" != 0 ]]; then
        # There was a problem in at least one execution of clang-format
        exit 1
    fi
}

# Function to run bazel formatting
function run_bazel_formatting () {
    echo "Running bazel buildifier to format all bazel files..."
    cd $BAZEL_ROOT_DIR
    bazel run @com_github_bazelbuild_buildtools//:buildifier

    if [[ "$?" != 0 ]]; then
        exit 1
    fi
}

# Function to run black python formatting
function run_black_formatting () {
    echo "Running black to format Python files..."
    $CURR_DIR/black_$BLACK_VERSION $BAZEL_ROOT_DIR

    if [[ "$?" != 0 ]]; then
        exit 1
    fi
}

# Run formatting
run_clang_format
run_bazel_formatting
run_black_formatting

exit 0
