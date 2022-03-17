#!/opt/tbotspython/bin/python3.8

import os
import sys
import iterfzf
from subprocess import PIPE, run, check_call
import argparse
from thefuzz import fuzz
from thefuzz import process

THEFUZZ_MATCH_RATIO_THRESHOLD = 50

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Run stuff")

    parser.add_argument("action", choices=["build", "run", "test"])
    parser.add_argument("search_query")
    parser.add_argument("-v", "--enable_visualizer", action="store_true")
    parser.add_argument("-s", "--stop_ai_on_start", action="store_true")
    parser.add_argument("-p", "--print_command", action="store_true")
    parser.add_argument("-d", "--debug_build", action="store_true")

    args, unknown_args = parser.parse_known_args()

    bazel_queries = {
        "test": ["bazel", "query", "tests(//...)"],
        "run": ["bazel", "query", "kind(.*_binary,//...)"],
        "build": ["bazel", "query", "kind(.*_library,//...)"],
    }

    # Run the appropriate bazel query and ask thefuzz to find the best matching
    # target, gauranteed to return 1 result because we set limit=1
    targets = run(bazel_queries[args.action], stdout=PIPE).stdout.split(b"\n")
    target, confidence = process.extract(args.search_query, targets, limit=1)[0]
    target = str(target, encoding="utf-8")

    # If the fuzz is confident, then just run it
    if confidence > THEFUZZ_MATCH_RATIO_THRESHOLD:
        print("Found target {} with confidence {}".format(target, confidence))

    # If not, launch FZF to let the user find the target
    else:
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

    # Handle stop_ai_on_start
    if args.stop_ai_on_start:
        if args.action in "run":
            command += ["--enable_visualizer"]
        if args.action in "test":
            command += ['--test_arg="--stop_ai_on_start"']

    # Handle visualizer argument
    if args.enable_visualizer:
        if args.action in "run":
            command += ["--enable_visualizer"]
        if args.action in "test":
            command += ['--test_arg="--enable_visualizer"']

    command += unknown_args

    # If the user requested a command dump, just print the command to run
    if args.print_command:
        print(" ".join(command))

    # Otherwise, run the command! We use os.system here because we don't
    # care about the output and subprocess doesn't seem to run qt for somereason
    else:
        os.system(" ".join(command))
