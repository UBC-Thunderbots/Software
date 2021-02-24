#!/bin/bash

#
# This script is used for running formatting checks in CI
#

# The version of the clang executable to use
export CLANG_VERSION=10.0
# The version of black to use
export BLACK_VERSION=19-10b0

# The directory this script is in
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The root bazel directory
BAZEL_ROOT_DIR="$CURR_DIR/../src"

# Extensions to check formatting for clang-format
CLANG_FORMAT_EXTENSIONS=(h cpp c hpp tpp proto)

# Function to run clang-format
function run_clang_format () {
    printf "Running clang-format over all files...\n\n"
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
        printf "\n***Failed to run clang-format over all files!***\n\n"
        exit 1
    fi
}

# Function to run bazel formatting
function run_bazel_formatting () {
    printf "Running bazel buildifier to format all bazel BUILD files...\n\n"
    cd $BAZEL_ROOT_DIR
    bazel run @com_github_bazelbuild_buildtools//:buildifier

    if [[ "$?" != 0 ]]; then
        printf "\n***Failed to format bazel BUILD files!***\n\n"
        exit 1
    else
        # extra new line to make it look nicer
        printf "\n"
    fi
}

# Function to run black python formatting
function run_black_formatting () {
    printf "Running black to format Python files...\n\n"
    # suppress misleading "All done! ✨ 🍰 ✨" message
    $CURR_DIR/black_$BLACK_VERSION $BAZEL_ROOT_DIR &>/dev/null

    if [[ "$?" != 0 ]]; then
        printf "\n***Failed to format Python files!***\n\n"
        exit 1
    fi
}

function run_code_spell(){
    printf "Fixing spelling...\n\n"
    cd $CURR_DIR/../src/software && codespell -w --skip="1,2,0" # Skip binaries
    cd $CURR_DIR/../src/firmware_new && codespell -w
    cd $CURR_DIR/../src/firmware/app && codespell -w
    cd $CURR_DIR/../src/shared && codespell -w
    cd $CURR_DIR/../docs && codespell -w --skip="*.png" # Skip images

    if [[ "$?" != 0 ]]; then
        printf "\n***Failed to fix spelling!***\n\n"
        exit 1
    fi
}

function run_git_diff_check(){
    printf "Checking for merge conflict markers...\n\n"
    cd $CURR_DIR && git --no-pager diff --check
    if [[ "$?" != 0 ]]; then
        printf "***Please fix merge conflict markers!***\n\n"
        exit 1
    fi
}

function run_eof_new_line(){
    printf "Adding missing new lines to end of files...\n\n"

    # adds missing new lines to the end of non-binary files
    cd $CURR_DIR/../ && git grep -zIl '' | while IFS= read -rd '' f; do tail -c1 < "$f" | read -r _ || echo >> "$f"; done
    if [[ "$?" != 0 ]]; then
        printf "***Failed to add missing new lines!***\n\n"
        exit 1
    fi
}

# Run formatting
#run_code_spell
run_clang_format
run_bazel_formatting
run_black_formatting
run_eof_new_line
run_git_diff_check

exit 0
