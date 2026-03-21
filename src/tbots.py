#!/opt/tbotspython/bin/python3

import itertools
import os
import sys
from dataclasses import dataclass
from enum import Enum
from subprocess import PIPE, run

import iterfzf
import questionary
from thefuzz import process
from typer import Argument, Context, Typer

from cli.cli_params import (
    ActionArgument,
    AnsiblePlaybook,
    DebugBinary,
    DebugBuildOption,
    EnableThunderscopeOption,
    EnableVisualizerOption,
    FlashRobotsOption,
    InteractiveCLI,
    InteractiveModeOption,
    JobsOption,
    NoOptimizedBuildOption,
    PrintCommandOption,
    RobotName,
    SearchQueryArgument,
    SelectDebugBinariesOption,
    SSHPasswordOption,
    StopAIOnStartOption,
    TestSuiteOption,
    TracyOption,
)

# thefuzz is a fuzzy string matcher in python
# https://github.com/seatgeek/thefuzz
#
# It returns a match ratio between the input and the choices
# This is an experimentally determined threshold that works
# for our bazel commands
THEFUZZ_MATCH_RATIO_THRESHOLD = 50
NUM_FILTERED_MATCHES_TO_SHOW = 10


@dataclass
class BuildOptions:
    action: ActionArgument
    search_query: str | None
    no_optimized_build: bool
    debug_build: bool
    select_debug_binaries: list | None
    flash_robots: list | None
    ssh_password: str | None
    interactive_search: bool
    tracy: bool
    test_suite: bool
    enable_thunderscope: bool
    enable_visualizer: bool
    stop_ai_on_start: bool
    jobs_option: str | None
    robot_name: str | None
    ansible_playbook: str | None


class BazelFlag(tuple, Enum):
    DEBUG_BUILD    = ("-c", "dbg")
    OPTIMIZED      = ("--copt=-O3",)
    ROBOT_PLATFORM = ("--platforms=//toolchains/cc:robot",)
    TRACY          = ("--cxxopt=-DTRACY_ENABLE",)
    THUNDERSCOPE   = ("--spawn_strategy=local", "--test_env=DISPLAY=:0")
    NO_CACHE_TESTS = ("--cache_test_results=false",)


app = Typer()


@app.command(
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
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

    validate(action, search_query, flash_robots, ssh_password, ansible_playbook, test_suite)

    command = create_command(
        ctx, action, search_query, no_optimized_build, debug_build,
        select_debug_binaries, flash_robots, ssh_password, interactive_search,
        tracy, test_suite, enable_thunderscope, enable_visualizer,
        stop_ai_on_start, jobs_option, robot_name, ansible_playbook,
    )

    if print_command:
        print(" ".join(command))
    else:
        print(
            "\n================================= Running: ======================================\n"
        )
        print(" ".join(command))
        print(
            "\n=================================================================================\n"
        )
        code = os.system(" ".join(command))
        sys.exit(1 if code != 0 else 0)


def validate(action, search_query, flash_robots, ssh_password, ansible_playbook, test_suite):
    if action is None:
        print("Error: 'action' is required unless using --interactive-cli")
        sys.exit(1)
    if bool(flash_robots) or bool(ansible_playbook):
        if not ssh_password:
            print(
                "If you want to flash robots or run ansible playbooks, both the robot IDs and password must be provided"
            )
            sys.exit(1)
    if search_query is None and (not test_suite or action != ActionArgument.test):
        print(
            "You must specify a search query unless you are running the test suite, use ./tbots.py test --suite instead"
        )
        sys.exit(1)


def create_command(
    ctx, action, search_query, no_optimized_build, debug_build,
    select_debug_binaries, flash_robots, ssh_password, interactive_search,
    tracy, test_suite, enable_thunderscope, enable_visualizer,
    stop_ai_on_start, jobs_option, robot_name, ansible_playbook,
):
    if test_suite and action == ActionArgument.test:
        target = """-- //...                              \\
                      -//software/field_tests/...         \\
                      -//toolchains/cc/...                \\
                      -//software:unix_full_system_tar_gen"""
        print("Running software and simulated gameplay test suite")
    else:
        target = fuzzy_find_target(action, search_query, interactive_search)

    command = ["bazel", action.value]
    unknown_args = ctx.args

    flag_conditions = {
        BazelFlag.DEBUG_BUILD:    debug_build or bool(select_debug_binaries),
        BazelFlag.OPTIMIZED:      not debug_build and (not no_optimized_build or bool(flash_robots)),
        BazelFlag.ROBOT_PLATFORM: bool(flash_robots or ansible_playbook),
        BazelFlag.TRACY:          tracy,
        BazelFlag.THUNDERSCOPE:   enable_thunderscope,
        BazelFlag.NO_CACHE_TESTS: action == ActionArgument.test,
    }
    for flag, condition in flag_conditions.items():
        if condition:
            command += list(flag.value)

    if jobs_option:
        command += ["--jobs=" + jobs_option]

    if select_debug_binaries:
        if DebugBinary.sim in select_debug_binaries:
            unknown_args += ["--debug_simulator"]
        if DebugBinary.blue in select_debug_binaries:
            unknown_args += ["--debug_blue_full_system"]
        if DebugBinary.yellow in select_debug_binaries:
            unknown_args += ["--debug_yellow_full_system"]

    command += [target]
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
        if not robot_name or not ssh_password:
            print("please provided robot name and ssh password")
            sys.exit(1)
        bazel_arguments += ["--playbook", ansible_playbook]
        bazel_arguments += ["--hosts", f"{robot_name}.local"]
        bazel_arguments += ["-pwd", ssh_password]

    if flash_robots:
        bazel_arguments += ["-pb deploy_robot_software.yml"]
        bazel_arguments += ["--hosts"]
        bazel_arguments += [f"192.168.6.20{id}" for id in flash_robots]
        bazel_arguments += ["-pwd", ssh_password]

    if action == ActionArgument.test:
        if (
            "--debug_blue_full_system" in unknown_args
            or "--debug_yellow_full_system" in unknown_args
            or "--debug_simulator" in unknown_args
        ):
            print(
                "Do not run simulated pytests as a test when debugging, use ./tbots.py -d run instead"
            )
            sys.exit(1)
        command += ['--test_arg="' + arg + '"' for arg in bazel_arguments]
    else:
        command += bazel_arguments

    return command


def start_interactive_cli():
    command = [sys.executable, sys.argv[0]]

    category = questionary.select(
        "What would you like to do?", choices=["Run thunderscope", "Test", "Flash"]
    ).ask()

    match category:
        case "Run thunderscope":
            command.extend(["run", "thunderscope"])
            launch_option = questionary.select(
                "How would you like to launch thunderscope?",
                choices=["Simulator", "Diagnostics"],
            ).ask()
            match launch_option:
                case "Simulator":
                    autoref = questionary.confirm("Enable autoref?", default=False).ask()
                    ci_mode = questionary.confirm("Enable ci_mode?", default=False).ask()
                    if autoref:
                        command.append("--enable_autoref")
                    if ci_mode:
                        command.append("--ci_mode")
                case "Diagnostics":
                    interface = questionary.text("What is your network interface?").ask()
                    command.extend(["--run_diagnostics", "--interface", interface])
        case "Test":
            test = questionary.text(
                "Please enter a test name (leave empty for entire suite)"
            ).ask()
            test = "all" if not test else test
            command.extend(["test", "--suite" if test == "all" or test == "" else test])
        case "Flash":
            playbook = questionary.select(
                "Please select an ansible playbook:",
                choices=["setup_pi.yml", "deploy_robot_software.yml", "deploy_powerboard.yml"],
            ).ask()
            robot = questionary.text("Please enter robot name (e.g. balle)").ask()
            password = questionary.password("Please enter ssh password").ask()
            command.extend(
                ["run", "ansible", "-ap", playbook, "-rn", robot, "-pwd", password]
            )
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
    test_query    = ["bazel", "query", "tests(//...)"]
    binary_query  = ["bazel", "query", "kind(.*_binary,//...)"]
    library_query = ["bazel", "query", "kind(.*_library,//...)"]
    bazel_queries = {
        ActionArgument.test:  [test_query],
        ActionArgument.run:   [test_query, binary_query],
        ActionArgument.build: [library_query, test_query, binary_query],
    }

    # Run the appropriate bazel query and ask thefuzz to find the best matching
    # target, guaranteed to return 1 result because we set limit=1
    # Combine results of multiple queries with itertools.chain
    targets = list(
        itertools.chain.from_iterable(
            run(query, stdout=PIPE).stdout.rstrip(b"\n").split(b"\n")
            for query in bazel_queries[action]
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
        targets = [target_dict[name] for name, _ in filtered_targets]
        target = str(iterfzf.iterfzf(iter(targets)), encoding="utf-8")
        print("User selected {}".format(target))

    return target


if __name__ == "__main__":
    app()
