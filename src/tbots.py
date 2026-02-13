#!/opt/tbotspython/bin/python3

import os
import sys

import iterfzf
import itertools
from subprocess import PIPE, run
from thefuzz import process
from typer import Typer, Context
from cli.cli_params import (
    ActionArgument,
    PrintCommandOption,
    NoOptimizedBuildOption,
    DebugBuildOption,
    SelectDebugBinariesOption,
    FlashRobotsOption,
    SSHPasswordOption,
    InteractiveModeOption,
    TracyOption,
    PlatformOption,
    EnableThunderscopeOption,
    EnableVisualizerOption,
    StopAIOnStartOption,
    SearchQueryArgument,
    TestSuiteOption,
    DebugBinary,
    Platform,
)

# thefuzz is a fuzzy string matcher in python
# https://github.com/seatgeek/thefuzz
#
# It returns a match ratio between the input and the choices
# This is an experimentally determined threshold that works
# for our bazel commands
THEFUZZ_MATCH_RATIO_THRESHOLD = 50
NUM_FILTERED_MATCHES_TO_SHOW = 10

app = Typer()


@app.command(
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
    no_args_is_help=True,
)
def main(
    ctx: Context,
    action: ActionArgument,
    search_query: SearchQueryArgument = None,
    print_command: PrintCommandOption = False,
    no_optimized_build: NoOptimizedBuildOption = False,
    debug_build: DebugBuildOption = False,
    select_debug_binaries: SelectDebugBinariesOption = None,
    flash_robots: FlashRobotsOption = None,
    ssh_password: SSHPasswordOption = None,
    interactive_search: InteractiveModeOption = False,
    tracy: TracyOption = False,
    platform: PlatformOption = None,
    enable_thunderscope: EnableThunderscopeOption = False,
    enable_visualizer: EnableVisualizerOption = False,
    stop_ai_on_start: StopAIOnStartOption = False,
    test_suite: TestSuiteOption = False,
) -> None:
    if bool(flash_robots) ^ bool(ssh_password):
        print(
            "If you want to flash robots, both the robot IDs and password must be provided"
        )
        sys.exit(1)

    if search_query is None and (not test_suite or not action == ActionArgument.test):
        print(
            "You must specify a search query unless you are running the test suite, use ./tbots.py test --suite instead"
        )
        sys.exit(1)

    test_query = ["bazel", "query", "tests(//...)"]
    binary_query = ["bazel", "query", "kind(.*_binary,//...)"]
    library_query = ["bazel", "query", "kind(.*_library,//...)"]

    bazel_queries = {
        "test": [test_query],
        "run": [test_query, binary_query],
        "build": [library_query, test_query, binary_query],
    }

    # Run the appropriate bazel query and ask thefuzz to find the best matching
    # target, guaranteed to return 1 result because we set limit=1
    # Combine results of multiple queries with itertools.chain
    targets = (
        list(
            itertools.chain.from_iterable(
                [
                    run(query, stdout=PIPE).stdout.rstrip(b"\n").split(b"\n")
                    for query in bazel_queries[action]
                ]
            )
        )
        if not test_suite
        else []
    )
    # Create a dictionary to map target names to complete bazel targets
    target_dict = (
        {target.split(b":")[-1]: target for target in targets}
        if not test_suite
        else {
            b"simulated_gameplay_tests": b"""//software:unix_full_system    \\
            //software/simulated_tests/... \\
            //software/ai/hl/...           \\
            //software/ai/navigator/...""",
            b"software_tests": b"""-- //... -//software:unix_full_system \\
              -//software/simulated_tests/...     \\
              -//software/ai/hl/...               \\
              -//software/field_tests/...         \\
              -//software/ai/navigator/...        \\
              -//toolchains/cc/...                \\
              -//software:unix_full_system_tar_gen""",
        }
    )

    # Use thefuzz to find the best matching target name
    most_similar_target_name, confidence = process.extract(
        search_query, list(target_dict.keys()), limit=1
    )[0]
    target = str(target_dict[most_similar_target_name], encoding="utf-8")

    print("Found target {} with confidence {}".format(target, confidence))

    if interactive_search or confidence < THEFUZZ_MATCH_RATIO_THRESHOLD:
        filtered_targets = process.extract(
            search_query,
            list(target_dict.keys()),
            limit=NUM_FILTERED_MATCHES_TO_SHOW,
        )
        targets = [
            target_dict[filtered_target_name[0]]
            for filtered_target_name in filtered_targets
        ]
        target = str(iterfzf.iterfzf(iter(targets)), encoding="utf-8")
        print("User selected {}".format(target))

    command = ["bazel", action.value]
    unknown_args = ctx.args

    # Trigger a debug build
    if debug_build or select_debug_binaries:
        command += ["-c", "dbg"]

    # Trigger an optimized build by default. Note that Thunderloop should always be
    # compiled with optimizations for best performance
    if not debug_build and (not no_optimized_build or flash_robots):
        command += ["--copt=-O3"]

    # Used for when flashing Jetsons
    if flash_robots:
        command += ["--platforms=//toolchains/cc:robot"]

    # Select debug binaries to run
    if select_debug_binaries:
        if DebugBinary.sim in select_debug_binaries:
            unknown_args += ["--debug_simulator"]
        if DebugBinary.blue in select_debug_binaries:
            unknown_args += ["--debug_blue_full_system"]
        if DebugBinary.yellow in select_debug_binaries:
            unknown_args += ["--debug_yellow_full_system"]

    # To run the Tracy profile, enable the TRACY_ENABLE macro
    if tracy:
        command += ["--cxxopt=-DTRACY_ENABLE"]

    if platform:
        command += ["--//software/embedded:host_platform=" + platform.value]

    # Don't cache test results
    if action == ActionArgument.test:
        command += ["--cache_test_results=false"]
    if action == ActionArgument.run:
        command += ["--"]

    command.append(target)

    bazel_arguments = unknown_args
    if stop_ai_on_start:
        bazel_arguments += ["--stop_ai_on_start"]
    if enable_visualizer:
        bazel_arguments += ["--enable_visualizer"]
    if enable_thunderscope:
        bazel_arguments += ["--enable_thunderscope"]
    if flash_robots:
        if not platform:
            print("No platform specified! Make sure to set the --platform argument.")
            sys.exit(1)
        bazel_arguments += ["-pb deploy_robot_software.yml"]
        bazel_arguments += ["--hosts"]
        platform_ip = "0" if platform == Platform.NANO else "6"
        bazel_arguments += [f"192.168.{platform_ip}.20{id}" for id in flash_robots]
        bazel_arguments += ["-pwd", ssh_password]

    if action == ActionArgument.test:
        command += ['--test_arg="' + arg + '"' for arg in bazel_arguments]

        if (
            "--debug_blue_full_system" in unknown_args
            or "--debug_yellow_full_system" in unknown_args
            or "--debug_simulator" in unknown_args
        ):
            print(
                "Do not run simulated pytests as a test when debugging, use ./tbots.py -d run instead"
            )
            sys.exit(1)

    else:
        command += bazel_arguments

    # If the user requested a command dump, just print the command to run
    if print_command:
        print(" ".join(command))

    # Otherwise, run the command! We use os.system here because we don't
    # care about the output and subprocess doesn't seem to run qt for somereason
    else:
        print(" ".join(command))
        code = os.system(" ".join(command))
        # propagate exit code
        sys.exit(1 if code != 0 else 0)


if __name__ == "__main__":
    app()
