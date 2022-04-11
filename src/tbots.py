#!/opt/tbotspython/bin/python3.8

import os
import sys
import iterfzf
import itertools
from subprocess import PIPE, run, check_call
import argparse
from thefuzz import fuzz
from thefuzz import process

# thefuzz is a fuzzy string matcher in python
# https://github.com/seatgeek/thefuzz
#
# It returns a match ratio between the input and the choices
# This is an experimentally determined threshold that works
# for our bazel commands
THEFUZZ_MATCH_RATIO_THRESHOLD = 50
NUM_FILTERED_MATCHES_TO_SHOW = 10

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run stuff")

    parser.add_argument("action", choices=["build", "run", "test"])
    parser.add_argument("search_query")
    parser.add_argument("-p", "--print_command", action="store_true")
    parser.add_argument("-d", "--debug_build", action="store_true")
    parser.add_argument("-i", "--interactive", action="store_true")

    # These are shortcut args for commonly used arguments on our tests
    # and full_system. All other arguments are passed through as-is
    # to the underlying binary/test that is being run (unknown_args)
    parser.add_argument("-t", "--enable_thunderscope", action="store_true")
    parser.add_argument("-v", "--enable_visualizer", action="store_true")
    parser.add_argument("-s", "--stop_ai_on_start", action="store_true")
    args, unknown_args = parser.parse_known_args()

    test_query = ["bazel", "query", "tests(//...)"]
    binary_query = ["bazel", "query", "kind(.*_binary,//...)"]
    library_query = ["bazel", "query", "kind(.*_library,//...)"]

    bazel_queries = {
        "test": [test_query],
        "run": [test_query, binary_query],
        "build": [library_query, test_query, binary_query],
    }

    # Run the appropriate bazel query and ask thefuzz to find the best matching
    # target, gauranteed to return 1 result because we set limit=1
    # Combine results of multiple queries with itertools.chain
    targets = list(
        itertools.chain.from_iterable(
            [
                run(query, stdout=PIPE).stdout.split(b"\n")
                for query in bazel_queries[args.action]
            ]
        )
    )
    target, confidence = process.extract(args.search_query, targets, limit=1)[0]
    target = str(target, encoding="utf-8")

    print("Found target {} with confidence {}".format(target, confidence))

    if args.interactive or confidence < THEFUZZ_MATCH_RATIO_THRESHOLD:
        filtered_targets = process.extract(
            args.search_query, targets, limit=NUM_FILTERED_MATCHES_TO_SHOW
        )
        targets = [filtered_target[0] for filtered_target in filtered_targets]
        target = str(iterfzf.iterfzf(iter(targets)), encoding="utf-8")
        print("User selected {}".format(target))

    command = ["bazel", args.action, target]

    # Trigger a debug build
    if args.debug_build:
        command += ["-c", "dbg"]

    # If its a binary, then run under gdb
    if args.action in "run" and args.debug_build:
        command += ["--run_under=gdb"]

    # Don't cache test results
    if args.action in "test":
        command += ["--cache_test_results=false"]
    if args.action in "run":
        command += ["--"]

    bazel_arguments = unknown_args

    if args.stop_ai_on_start:
        bazel_arguments += ["--stop_ai_on_start"]
    if args.enable_visualizer:
        bazel_arguments += ["--enable_visualizer"]
    if args.enable_thunderscope:
        bazel_arguments += ["--enable_thunderscope"]

    if args.action in "test":
        command += ['--test_arg="' + arg + '"' for arg in bazel_arguments]
    else:
        command += bazel_arguments

    # If the user requested a command dump, just print the command to run
    if args.print_command:
        print(" ".join(command))

    # Otherwise, run the command! We use os.system here because we don't
    # care about the output and subprocess doesn't seem to run qt for somereason
    else:
        print(" ".join(command))
        os.system(" ".join(command))
