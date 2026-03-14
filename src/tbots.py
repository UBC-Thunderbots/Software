#!/opt/tbotspython/bin/python3

import os
import sys

import iterfzf
import itertools
from subprocess import PIPE, run
from thefuzz import process
from typer import Typer, Context, Argument, prompt
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
    EnableThunderscopeOption,
    EnableVisualizerOption,
    StopAIOnStartOption,
    SearchQueryArgument,
    TestSuiteOption,
    DebugBinary,
    JobsOption,
    RobotName,
    AnsiblePlaybook,
    InteractiveCLI,
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
    context_settings={"allow_extra_args": True,
                      "ignore_unknown_options": True},
    no_args_is_help=True,
)
def main(
    ctx: Context,
    action: ActionArgument = Argument(None),
    search_query: str = Argument(None),
    print_command: PrintCommandOption = False,
    no_optimized_build: NoOptimizedBuildOption = False,
    debug_build: DebugBuildOption = False,
    select_debug_binaries: SelectDebugBinariesOption = None,
    flash_robots: FlashRobotsOption = None,
    ssh_password: SSHPasswordOption = None,
    interactive_search: InteractiveModeOption = False,
    tracy: TracyOption = False,
    test_suite: TestSuiteOption = False,
    enable_thunderscope: EnableThunderscopeOption = False,
    enable_visualizer: EnableVisualizerOption = False,
    stop_ai_on_start: StopAIOnStartOption = False,
    jobs_option: JobsOption = None,
    robot_name: RobotName = None,
    ansible_playbook: AnsiblePlaybook = None,
    interactive_cli: InteractiveCLI = False,
) -> None:
    if interactive_cli:
        start_interactive_cli()
        return
    if action is None:
        print(
            "Error: 'action' is required unless using --interactive-cli")
        sys.exit(1)
    if bool(flash_robots) or bool(ansible_playbook):
        if not ssh_password:
            print(
                "If you want to flash robots or run ansible playbooks, both the robot IDs and password must be provided"
            )
            sys.exit(1)
        isFlashing = True

    if search_query is None and (not test_suite or not action == ActionArgument.test):
        print(
            "You must specify a search query unless you are running the test suite, use ./tbots.py test --suite instead"
        )
        sys.exit(1)

    if test_suite and action == ActionArgument.test:
        target = """-- //...                              \\
                      -//software/field_tests/...         \\
                      -//toolchains/cc/...                \\
                      -//software:unix_full_system_tar_gen"""
        print("Running software and simulated gameplay test suite")
    else:
        target = fuzzy_find_target(action, search_query, interactive_search)

    command = ["bazel", action.value, target]
    unknown_args = ctx.args

    # Trigger a debug build
    if debug_build or select_debug_binaries:
        command += ["-c", "dbg"]

    # Trigger an optimized build by default. Note that Thunderloop should always be
    # compiled with optimizations for best performance
    if not debug_build and (not no_optimized_build or flash_robots):
        command += ["--copt=-O3"]

    # Used for when flashing Raspberry Pi
    if flash_robots or ansible_playbook:
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

    # limit number of jobs
    if jobs_option:
        command += ["--jobs=" + jobs_option]

    if enable_thunderscope:
        command += ["--spawn_strategy=local", "--test_env=DISPLAY=:0"]

    # Don't cache test results
    if action == ActionArgument.test:
        command += ["--cache_test_results=false"]
    if action == ActionArgument.run:
        command += ["--"]

    bazel_arguments = unknown_args
    if stop_ai_on_start:
        bazel_arguments += ["--stop_ai_on_start"]
    if enable_visualizer:
        bazel_arguments += ["--enable_visualizer"]
    if enable_thunderscope:
        bazel_arguments += ["--enable_thunderscope"]

    if ansible_playbook:
        print(robot_name)
        print(ssh_password)
        print(ansible_playbook)
        if not robot_name or not ssh_password:
            print("please provided robot name and ssh password")
            sys.exit(1)
        bazel_arguments += ["--playbook", ansible_playbook]
        bazel_arguments += ["--hosts", f"robot@{robot_name}.local"]
        bazel_arguments += ["-pwd", ssh_password]

    if flash_robots:
        bazel_arguments += ["-pb deploy_robot_software.yml"]
        bazel_arguments += ["--hosts"]
        bazel_arguments += [f"192.168.6.20{
            id}" for id in flash_robots]
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
        print(command)
        # propagate exit code
        sys.exit(1 if code != 0 else 0)


def start_interactive_cli():
    goal = prompt(
        "Please choose a number: 1. test  2. flash  3. run thunderscope ")
    # Use sys.executable to ensure we use the same python interpreter
    command = [sys.executable, sys.argv[0]]

    match goal:
        case '1':
            test = prompt("Please enter a test name ('all' for entire suite)")
            command.extend(["test", '--suite' if test == 'all' else test])
        case '2':
            playbook = prompt(
                "Please enter ansible playbook name (e.g. setup_pi.yml)")
            robot = prompt("Please enter robot name (e.g. Balle)")
            password = prompt("Please enter ssh password", hide_input=True)
            # Match the argument flags defined in your cli_params
            command.extend(["run", "ansible", "-ap", playbook,
                           "-rn", robot, "-pwd", password])
        case '3':
            print("How would you like to launch thunderscope?")
            command.extend(["run", "thunderscope"])
            launch_option = prompt("1. Simulator  2. diagnostics")
            match launch_option:
                case '1':
                    autoref = prompt("Enable autoref? [y/N]").lower() == 'y'
                    ci_mode = prompt("Enable ci_mode? [y/N]").lower() == 'y'
                    if autoref:
                        command.append('--enable_autoref')
                    if ci_mode:
                        command.append('--ci_mode')
                case '2':
                    interface = prompt("What is your network interface?")
                    command.extend(
                        ['--run_diagnostics', '--interface', interface])
                case _:
                    print("Invalid option! Restarting...")
                    return start_interactive_cli()
        case _:
            print("Invalid option!")
            return start_interactive_cli()

    print(f"Executing: {' '.join(command)}")
    code = os.system(" ".join(command))
    sys.exit(1 if code != 0 else 0)


def fuzzy_find_target(
    action: ActionArgument,
    search_query: SearchQueryArgument,
    interactive_search: InteractiveModeOption,
) -> str:
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
    targets = list(
        itertools.chain.from_iterable(
            [
                run(query, stdout=PIPE).stdout.rstrip(b"\n").split(b"\n")
                for query in bazel_queries[action]
            ]
        )
    )
    # Create a dictionary to map target names to complete bazel targets
    target_dict = {target.split(b":")[-1]: target for target in targets}

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

    return target


if __name__ == "__main__":
    app()
