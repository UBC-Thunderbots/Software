#
# This script is used for running formatting checks in CI
#

CLANG_VERSION="7.0"
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "=========================================================="
echo " THIS SHOULD ONLY BE RUN IN CI, PLEASE DO NOT USE LOCALLY"
echo "=========================================================="

# Determine what we should compare this branch against to figure out what
# files were changed
if [ "$TRAVIS_PULL_REQUEST" == "false" ] ; then
  # Not in a pull request, so compare against parent commit
  BASE_COMMIT="HEAD^"
  echo "Running clang-format against parent commit $(git rev-parse $BASE_COMMIT)"
  echo "=================================================="
else
  # In a pull request so compare against branch we're trying to merge into
  # (ex. "master")
  BASE_COMMIT="$TRAVIS_BRANCH"
  # Make sure we pull the branches we're trying to merge against
  git fetch origin $BASE_COMMIT:$BASE_COMMIT
  echo "Running clang-format against branch $BASE_COMMIT, with hash $(git rev-parse $BASE_COMMIT)"
  echo "=================================================="
fi

# Run formatting
OUTPUT="$($CURR_DIR/fix_formatting.sh -b $BASE_COMMIT)"
FORMATTING_RETURN_VAL=$?

# Check if we changed any files (based on the return code of the last command)
if [ $FORMATTING_RETURN_VAL -ne 0 ]; then
    echo "$OUTPUT"
    echo "========================================================================"
    echo "clang-format failed :( - please reformat your code via the \`fix_formatting.sh\` script and resubmit"
    exit 1
else
    echo "clang-format passed, no files changed :D"
    exit 0
fi
