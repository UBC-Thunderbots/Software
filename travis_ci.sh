#!/bin/bash

#############################################################################
# This script is what will run in CI (Continuous Integration) to make sure  #
# the code base is building, tests are passing, and formatting is correct   #
#                                                                           #
# It relies on certain bash variables being set by CI before this script is #
# called. This setup is dictated in the `.travis.yml` file                  # 
#############################################################################

#TODO: We use the coverage arguments for CMake multiple times here, so we should put them in a variables!!!n
# TODO: conditionals should reflect the fact that we must build and run tests to get coverage

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

# Note that we must build the codebase in order to run tests
if [ "$RUN_BUILD" == "true" ] || [ "$RUN_TESTS" == "true" ]; then
    # Install all required dependecies
    travis_run ./environment_setup/setup_software.sh kinetic

    # Build the codebase
    if [ "$RUN_COVERAGE" == "true" ]; then
        # Build with coverage - slower but gives more detailed coverage info
        # that we will use later to produce a report
        travis_run catkin_make \
            -DCMAKE_BUILD_TYPE=Debug \
            -DCMAKE_CXX_FLAGS=\'-O0 -fprofile-arcs -ftest-coverage\' \
            -DCMAKE_CXX_OUTPUT_EXTENSION_REPLACE=1 \
            VERBOSE=1
    else
        # Build Normally
        travis_run catkin_make
    fi
fi

if [ "$RUN_TESTS" == "true" ]; then
    if [ "$RUN_COVERAGE" == "true" ]; then
        # Build with coverage - slower but gives more detailed coverage info
        # that we will use later to produce a report
        travis_run catkin_make run_tests \
            -DCMAKE_BUILD_TYPE=Debug \
            -DCMAKE_CXX_FLAGS=\'-O0 -fprofile-arcs -ftest-coverage\' \
            -DCMAKE_CXX_OUTPUT_EXTENSION_REPLACE=1 \
            VERBOSE=1
    else
        # Run tests normally
        travis_run catkin_make run_tests
    fi

    # Report the results of the tests
    # (which tests failed and why)
    travis_run catkin_test_results --verbose
fi

if [ "$RUN_COVERAGE" == "true" ]; then
    # Install the C++ Wrapper for Coveralls (Our Coverage Checker)
    sudo apt-get install python-pip -y
    travis_run pip install --user cpp-coveralls pyOpenSSL

    # Make sure we can find the coveralls executable
    PATH="$PATH:$HOME/.local/bin"

    # Run The Coverage Checker
    travis_run coveralls -t $COVERALLS_REPO_TOKEN
fi

if [ "$RUN_FORMATTING_CHECKS" == "true" ]; then
    CLANG_VERSION="4.0"

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

    # Check if we need to change any files
    output="$($CURR_DIR/clang_format/git-clang-format --binary $CURR_DIR/clang_format/clang-format-$CLANG_VERSION --commit $BASE_COMMIT --diff)"
    if [[ $output == *"no modified files to format"* ]] || [[ $output == *"clang-format did not modify any files"* ]] ; then
        echo "clang-format passed :D"
        exit 0
    else
        echo "$output"
        echo "=================================================="
        echo "clang-format failed :( - please reformat your code via the \`fix_formatting.sh\` script and resubmit"
        exit 1
    fi
fi


echo "CI Script has finished successfully!"
exit 0
