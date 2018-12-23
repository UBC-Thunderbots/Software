#!/bin/bash

# This script allows us to automatically format the entire codebase using
# clang-format, or some set of changes based on a diff via `git-clang-format`

# Will set the `FILES_CHANGED` environment variable to 1 if we changed any
# files, and 0 otherwise (assuming no error occured)

# Will return:
# `0` if no files formatted and no errors
# `1` if an unexpected error occured
# `3` if the script ran properly and we changed some files


# The version of the clang executable to use
export CLANG_VERSION=7.0

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The name of this script, used for help info
SCRIPT_NAME="$0"

# Extensions to check formatting
EXTENSIONS=(h cpp c hpp tpp)

# Check that we have at least some arguments
if (( $# == 0 )); then
    echo "Missing arguments (\"$SCRIPT_NAME --help\" for help)"
    exit 1
fi

# This loop iterates through and handles the provided command-line arguments.
# See: https://stackoverflow.com/questions/7069682/how-to-get-arguments-with-flags-in-bash-script/7069755?utm_medium=organic&utm_source=google_rich_qa&utm_campaign=google_rich_qa
#
# Arguments are handles one=by-one until there are none left. The 'shift'
# command shifts all arguments each time one has been handled
while test $# -gt 0; do
    case "$1" in
        -h|--help)
            echo "$SCRIPT_NAME - Run clang-format on our codebase"
            echo " "
            echo "$SCRIPT_NAME [options]"
            echo " "
            echo "options:"
            echo "-h, --help       show help"
            echo "-a, --all        format all files in our codebase"
            echo "-b, --branch     specify a git branch to diff against, and only format the files you have changed (NOTE: This argument will also accept a git commit hash)"
            echo " "
            echo "Examples:"
            echo "$SCRIPT_NAME -a"
            echo "$SCRIPT_NAME -b dev"

            exit 0
            ;;
        -a|--all)
            echo "Formatting all files..."

            # Generate extension string
            # Formatted as -iname *.EXTENSION -o
            EXTENSION_STRING=""
            for value in "${EXTENSIONS[@]}"
            do
                EXTENSION_STRING="$EXTENSION_STRING -iname *.$value -o"
            done
            
            # Find all the files that we want to format, and pass them to
            # clang-format as arguments
            # We remove the last -o flag from the extension string
            find $CURR_DIR/../src/ ${EXTENSION_STRING::-2} \
                | xargs $CURR_DIR/clang-format-$CLANG_VERSION -i -style=file

            shift
            ;;
        -b|--branch)
            # Shift the arguents to check for the argument to this flag
            shift
            # Make sure we only have 1 arguement for the branch name
            if test $# -lt 1; then
                echo "Error: No branch specified"
                exit 1
            elif test $# -gt 1; then
                echo "Error: More than one branch specified"
                exit 1
            fi

            # The name of the branch to diff against
            BRANCH_NAME="$1"

            # Generate extension string
            # Formatted as EXTENSION,
            EXTENSION_STRING=""
            for value in "${EXTENSIONS[@]}"
            do
                EXTENSION_STRING="$EXTENSION_STRING$value,"
            done

            # Fix formatting on all changes between this branch and the target branch
            # Remove the last comma from the extension string
            OUTPUT="$($CURR_DIR/git-clang-format --extensions ${EXTENSION_STRING::-1} --binary $CURR_DIR/clang-format-$CLANG_VERSION --commit $BRANCH_NAME)"

            # Check the results of clang-format
            if [[ $OUTPUT == *"no modified files to format"* ]] || [[ $OUTPUT == *"clang-format did not modify any files"* ]] ; then
                # No files were changed
                echo "clang-format passed, no files changed :D"
                exit 0
            else
                # We changed some files
                # Output the results.
                # We're using printf here so we can replace spaces with newlines to make the output readable
                printf '%s\n' $OUTPUT
                echo " "
                echo "========================================================================"
                echo "clang-format has modified the above files. Formatting complete."
                exit 3
            fi

            shift
            ;;
        *)
            echo "Invalid arguments (\"$SCRIPT_NAME --help\" for help)"
            exit 1
            ;;
    esac
done
