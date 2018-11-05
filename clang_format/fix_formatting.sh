#!/bin/bash

# The version of the clang executable to use
export CLANG_VERSION=7.0

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# The name of this script, used for help info
SCRIPT_NAME="$0"

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
            echo "-b, --branch     specify a git branch to diff against, and only format the files you have changed"
            echo " "
            echo "Examples:"
            echo "$SCRIPT_NAME -a"
            echo "$SCRIPT_NAME -b dev"

            exit 0
            ;;
        -a|--all)
            echo "Formatting all files..."
            
            # Find all the files that we want to format, and pass them to
            # clang-format as arguments
            find $CURR_DIR/../src/ -iname *.h -o -iname *.cpp -o -iname *.c \
                -o -iname *.hpp -o -iname *.tpp \
                | xargs $CURR_DIR/clang-format-$CLANG_VERSION -i -style=file

            shift
            ;;
        -b|--branch)
            # Shift the arguents to check for the argument to this flag
            shift
            # Make sure we only have 1 arguement for the branch name
            if test $# -ne 1; then
                echo "Error: No branch specified"
                exit 1
            fi

            # The name of the branch to diff against
            BRANCH_NAME="$1"

            # Fix formatting on all changes between this branch and the target branch
            OUTPUT="$($CURR_DIR/git-clang-format --extensions h, cpp, c, hpp, tpp --binary $CURR_DIR/clang-format-$CLANG_VERSION --commit $BRANCH_NAME)"

            # Check the results of clang-format
            if [[ $OUTPUT == *"no modified files to format"* ]] || [[ $OUTPUT == *"clang-format did not modify any files"* ]] ; then
                # Great, we passed!
                echo "clang-format passed, no files changed :D"
                exit 0
            else
                # Output the results.
                # We're using printf here so we can replace spaces with newlines to make the output readable
                printf '%s\n' $OUTPUT
                echo " "
                echo "========================================================================"
                echo "clang-format has modified the above files. Formatting complete."
            fi

            shift
            ;;
        *)
            echo "Missing or invalid arguments (\"$SCRIPT_NAME --help\" for help)"

            break
            ;;
    esac
done

