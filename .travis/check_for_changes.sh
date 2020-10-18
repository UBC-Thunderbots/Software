#!/bin/bash

# Adapted from https://github.com/arnaskro/monorepo-travis/blob/master/.travis/build-condition.sh

if [[ -z $1 ]]; then
    echo "Commit range cannot be empty"
    exit 1
fi

if [[ -z $2 ]]; then
    echo "Change path cannot be empty"
    exit 1
fi

git diff --name-only $1 | sort -u | uniq | grep $2 > /dev/null
