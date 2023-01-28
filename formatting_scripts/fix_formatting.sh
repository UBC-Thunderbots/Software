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
CLANG_FORMAT_EXTENSIONS=(h cpp c cc hpp tpp proto)

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

# Format the imports and remove unused imports/variables
# NOTE: It's important that this runs _before_ black formatting
function run_python_import_formatting(){
    /opt/tbotspython/bin/autoflake -r --in-place --remove-unused-variables $CURR_DIR/../src/software
}


# Function to run black python formatting
function run_black_formatting () {
    printf "Running black to format Python files...\n\n"

    $CURR_DIR/black_$BLACK_VERSION $BAZEL_ROOT_DIR

    if [[ "$?" != 0 ]]; then
        printf "\n***Failed to format Python files!***\n\n"
        exit 1
    fi
}

function run_code_spell(){
    mkdir -p $CURR_DIR/dictionary
    http_code=$(curl -sw '%{http_code}' https://raw.githubusercontent.com/codespell-project/codespell/v1.14.0/codespell_lib/data/dictionary.txt --output $CURR_DIR/dictionary/dictionary.txt)

    if [[ "$http_code" != 200 ]]; then
        printf "\n***Failed to download codespell dictionary!***\n\n"
	    if [ ! -f $CURR_DIR/dictionary/edited_dictionary.txt ]; then 
	        printf "\n***Failed to load codespell dictonary from cache!***\n\n"
	        exit 1
	    fi
	    printf "Loading Codespell dictionary from cache!\n\n"
    else
        sed "/atleast/d" $CURR_DIR/dictionary/dictionary.txt > $CURR_DIR/dictionary/edited_dictionary.txt #removing spell fixes that include the word 'atleast' from codespell dictionary 
        rm $CURR_DIR/dictionary/dictionary.txt #remove the original dictionary.txt
    fi

    printf "Fixing spelling...\n\n"
    cd $CURR_DIR/../src/software && codespell -w --skip="1,2,0" -D $CURR_DIR/dictionary/edited_dictionary.txt # Skip binaries
    cd $CURR_DIR/../src/shared && codespell -w -D $CURR_DIR/dictionary/edited_dictionary.txt
    cd $CURR_DIR/../docs && codespell -w --skip="*.png,*.svg" -D $CURR_DIR/dictionary/edited_dictionary.txt # Skip images

    if [[ "$?" != 0 ]]; then
        printf "\n***Failed to fix spelling!***\n\n"
        exit 1
    fi
}

function run_git_diff_check(){
    printf "Checking for merge conflict markers...\n\n"
    cd $CURR_DIR && git -c "core.whitespace=-trailing-space" --no-pager diff --check
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
run_code_spell
run_clang_format
run_bazel_formatting
run_python_import_formatting
run_black_formatting
run_eof_new_line
run_git_diff_check

exit 0
