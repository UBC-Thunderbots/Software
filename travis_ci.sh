#!/bin/bash

#############################################################################
# This script is what will run in CI (Continuous Integration) to make sure  #
# the code base is building, tests are passing, and formatting is correct   #
#                                                                           #
# It relies on certain bash variables being set by CI before this script is #
# called. This setup is dictated in the `.travis.yml` file                  # 
#############################################################################

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# This variable is used to let us show nice folds in travis
export TRAVIS_FOLD_COUNTER=1

# Display command in Travis console and fold output in dropdown section
function travis_run() {
  local command=$@

  echo -e "\e[0Ktravis_fold:start:command$TRAVIS_FOLD_COUNTER \e[34m$ $command\e[0m"
  # actually run command
  eval ${command} || exit 1 # kill build if error
  echo -e -n "\e[0Ktravis_fold:end:command$TRAVIS_FOLD_COUNTER\e[0m"

  let "TRAVIS_FOLD_COUNTER += 1"
}


# Change to the directory this script is in
cd $CURR_DIR

# Note that we must build the codebase in order to run tests and/or get coverage
if [ "$RUN_BUILD" == "true" ] || \
    [ "$RUN_TESTS" == "true" ] || \
    [ "$RUN_COVERAGE" == "true" ]; then
    # Install all required dependecies
    travis_run ./environment_setup/setup_software.sh

    # Build the codebase
    travis_run catkin_make
fi

# Note that we must run tests to get coverage
if [ "$RUN_TESTS" == "true" ] || \
    [ "$RUN_COVERAGE" == "true" ]; then
    
    if [ "$RUN_COVERAGE" == "true" ]; then
        # Run tests for AI with coverage
        travis_run catkin_make run_tests -DENABLE_COVERAGE=ON
    else
        # Run tests for AI normally
        travis_run catkin_make run_tests
    fi

    # Run tests for Corner Kick
    travis_run ./src/corner_kick/scripts/start_test.sh

    # Report the results of the tests
    # (which tests failed and why)
    travis_run catkin_test_results --verbose

    if [ "$RUN_COVERAGE" == "true" ]; then
        # Upload coverage reports
        # Note that we only grab a certain number of lines from the head
        # and tail (start and end) of the command to prevent this generating
        # a massive log in CI
        travis_run bash <(curl -s https://codecov.io/bash) | (head -n100 && tail -n100)
    fi
fi

# We need to run tests in order to get coverage
if [ "$RUN_FORMATTING_CHECKS" == "true" ]; then    
    CLANG_VERSION="7.0"

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
    OUTPUT="$($CURR_DIR/clang_format/fix_formatting.sh -b $BASE_COMMIT)"
    FORMATTING_RETURN_VAL=$?

    # Check if we changed any files (based on the return code of the last command)
    if [ $FORMATTING_RETURN_VAL -eq 3 ] || [ $FORMATTING_RETURN_VAL -eq 1 ]; then
        echo "$OUTPUT"
        echo "========================================================================"
        echo "clang-format failed :( - please reformat your code via the \`fix_formatting.sh\` script and resubmit"
        exit 1
    else
        echo "clang-format passed, no files changed :D"
        exit 0
    fi
fi


echo "CI Script has finished successfully!"
exit 0
