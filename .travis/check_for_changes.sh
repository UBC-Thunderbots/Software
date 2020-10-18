#!/bin/bash

# Checks against commit $1 for files changed that match $2 and recommends whether or not we should run tests.
# exit code 1 if tests should be run, 0 if not.
# Adapted from https://github.com/arnaskro/monorepo-travis/blob/master/.travis/build-condition.sh

if [[ -z $1 ]]; then
    echo "Commit range cannot be empty"
    exit 1
fi

if [[ -z $2 ]]; then
    echo "Change path cannot be empty"
    exit 1
fi

if git diff --name-only $1 | sort -u | uniq | grep -q $2;then echo "There are relevant changes, so we should run the tests" && exit 1; else echo "We should skip tests because there were no relevant changes" && exit 0; fi
