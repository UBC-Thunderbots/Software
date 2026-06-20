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
    CATEGORY_CHOICES,
    DEBUG_POWERLOOP_PLAYBOOK,
    INTERACTIVE_STYLE,
    LAUNCH_MODE_CHOICES,
    PLAYBOOK_CHOICES,
    THUNDERSCOPE_SIMULATOR_OPTION_CHOICES,
    ActionArgument,
    AnsiblePlaybook,
    DebugBinary,
    DebugBuildOption,
    EnableThunderscopeOption,
    FlashRobotsOption,
    InteractiveModeOption,
    JobsOption,
    NoOptimizedBuildOption,
    PrintCommandOption,
    RobotName,
    RunsOption,
    SelectDebugBinariesOption,
    SSHPasswordOption,
    StopAIOnStartOption,
    TestSuiteOption,
    TracyOption,
)

THEFUZZ_MATCH_RATIO_THRESHOLD = 50
NUM_FILTERED_MATCHES_TO_SHOW = 10


@dataclass
class BuildConfig:
    action: ActionArgument
    search_query: str | None = None
    no_optimized_build: bool = False
    debug_build: bool = False
    select_debug_binaries: list | None = None
    flash_robots: list | None = None
    ssh_password: str | None = None
    interactive_search: bool = False
    tracy: bool = False
    test_suite: bool = False
    enable_thunderscope: bool = False
    stop_ai_on_start: bool = False
    jobs_option: str | None = None
    runs: int | None = None
    robot_name: str | None = None
    ansible_playbook: str | None = None
    debug_powerloop: bool = False


class BazelFlag(tuple, Enum):
    DEBUG_BUILD = ("-c", "dbg")
    OPTIMIZED = ("--copt=-O3",)
    ROBOT_PLATFORM = ("--platforms=//toolchains/cc:robot",)
    TRACY = ("--cxxopt=-DTRACY_ENABLE",)
    THUNDERSCOPE = ("--spawn_strategy=local", "--test_env=DISPLAY=:0")
    NO_CACHE_TESTS = ("--cache_test_results=false",)
    DEBUG_POWERLOOP = ("--//software/power:debug_powerloop",)


app = Typer()


@app.command(
    context_settings={"allow_extra_args": True, "ignore_unknown_options": True},
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
    stop_ai_on_start: StopAIOnStartOption = False,
    jobs_option: JobsOption = None,
    runs: RunsOption = None,
    robot_name: RobotName = None,
    ansible_playbook: AnsiblePlaybook = None,
) -> None:
    """Entry point for the tbots CLI.

    Parses the command-line options into a :class:`BuildConfig`, then validates,
    builds, and executes the corresponding Bazel command. When invoked with no
    action and no search query, falls back to the interactive menu-driven CLI.

    :param ctx: Typer context carrying any unparsed pass-through args forwarded
        to the underlying Bazel target as runtime arguments
    :param action: the Bazel action to perform (build, test, or run)
    :param search_query: fuzzy search query used to resolve the Bazel target
    :param print_command: if True, print the generated command instead of running it
    :param no_optimized_build: compile without -O3 optimizations
    :param debug_build: compile with debug symbols (-c dbg)
    :param select_debug_binaries: binaries to launch separately in debug mode
    :param flash_robots: robot IDs to flash with the deploy_robot_software playbook
    :param ssh_password: password Ansible uses when SSHing into the robots
    :param interactive_search: enable interactive fuzzy target selection
    :param tracy: build with the TRACY_ENABLE macro defined
    :param test_suite: run the entire test suite instead of a single target
    :param enable_thunderscope: launch with Thunderscope enabled
    :param stop_ai_on_start: start the binary with the AI paused
    :param jobs_option: value passed to Bazel's --jobs flag
    :param runs: value passed to Bazel's --runs_per_test flag
    :param robot_name: hostname of the robot targeted by an Ansible playbook
    :param ansible_playbook: name of the Ansible playbook to run
    """
    if not action and not search_query:
        start_interactive_cli()
        return

    config = BuildConfig(
        action=action,
        search_query=search_query,
        no_optimized_build=no_optimized_build,
        debug_build=debug_build,
        select_debug_binaries=select_debug_binaries,
        flash_robots=flash_robots,
        ssh_password=ssh_password,
        interactive_search=interactive_search,
        tracy=tracy,
        test_suite=test_suite,
        enable_thunderscope=enable_thunderscope,
        stop_ai_on_start=stop_ai_on_start,
        jobs_option=jobs_option,
        runs=runs,
        robot_name=robot_name,
        ansible_playbook=ansible_playbook,
    )

    validate(config)
    command = create_command(config, ctx.args)
    execute_command(command, print_only=print_command)


def validate(config: BuildConfig):
    """Validate a BuildConfig, exiting with an error message if it is invalid.

    Enforces that flashing or running an Ansible playbook supplies an SSH
    password, and that a non-suite invocation supplies a search query.

    :param config: the build configuration to validate
    """
    if bool(config.flash_robots) or bool(config.ansible_playbook):
        if not config.ssh_password:
            print(
                "Error: SSH password is required for flashing or ansible playbooks. Please include the --pwd flag"
            )
            sys.exit(1)
    if config.search_query is None and (
        not config.test_suite or config.action != ActionArgument.test
    ):
        print("Error: Specify a search query or use --suite with test.")
        sys.exit(1)


def create_command(config: BuildConfig, extra_args: list[str]) -> list[str]:
    """Build the Bazel command list from a config and pass-through args.

    Resolves the target (or test suite), applies the Bazel flags implied by the
    config, and appends runtime arguments such as debug flags and Ansible
    playbook parameters. Runtime args are wrapped as --test_arg values for the
    test action and appended after a ``--`` separator for the run action.

    :param config: the validated build configuration
    :param extra_args: unparsed CLI args forwarded as runtime arguments
    :return: the Bazel command as a list of tokens, ready to be joined and run
    """
    if config.test_suite and config.action == ActionArgument.test:
        target = """-- //...                              \\
                      -//software/gameplay_tests/...      \\
                      -//toolchains/cc/...                \\
                      -//software:unix_full_system_tar_gen"""
    else:
        target = fuzzy_find_target(
            config.action, config.search_query, config.interactive_search
        )

    command = ["bazel", config.action.value]
    runtime_args = list(extra_args)

    # Apply Bazel Flags
    flag_conditions = {
        BazelFlag.DEBUG_BUILD: config.debug_build or bool(config.select_debug_binaries),
        BazelFlag.OPTIMIZED: not config.debug_build
        and (not config.no_optimized_build or bool(config.flash_robots)),
        BazelFlag.ROBOT_PLATFORM: bool(config.flash_robots or config.ansible_playbook),
        BazelFlag.TRACY: config.tracy,
        BazelFlag.THUNDERSCOPE: config.enable_thunderscope,
        BazelFlag.NO_CACHE_TESTS: config.action == ActionArgument.test,
        BazelFlag.DEBUG_POWERLOOP: config.debug_powerloop,
    }
    for flag, condition in flag_conditions.items():
        if condition:
            command += list(flag.value)

    if config.jobs_option:
        command += [f"--jobs={config.jobs_option}"]

    if config.runs:
        command += [f"--runs_per_test={config.runs}"]

    # Handle binary debugging flags
    if config.select_debug_binaries:
        if DebugBinary.sim in config.select_debug_binaries:
            runtime_args.append("--debug_simulator")
        if DebugBinary.blue in config.select_debug_binaries:
            runtime_args.append("--debug_blue_full_system")
        if DebugBinary.yellow in config.select_debug_binaries:
            runtime_args.append("--debug_yellow_full_system")

    command += [target]

    # Separator for runtime arguments
    if config.action == ActionArgument.run:
        command += ["--"]

    # Append runtime arguments
    if config.stop_ai_on_start:
        runtime_args.append("--stop_ai_on_start")
    if config.enable_thunderscope:
        runtime_args.append("--enable_thunderscope")

    if config.ansible_playbook:
        runtime_args += [
            "--playbook",
            config.ansible_playbook,
            "--hosts",
            f"{config.robot_name}.local",
            "-pwd",
            config.ssh_password,
        ]

    if config.flash_robots:
        runtime_args += ["--playbook", "deploy_robot_software.yml", "--hosts"]
        runtime_args += [f"192.168.6.{200 + int(id)}" for id in config.flash_robots]
        runtime_args += ["-pwd", config.ssh_password]

    if config.action == ActionArgument.test:
        # Safety check for pytest debugging
        if any(
            x in runtime_args
            for x in [
                "--debug_blue_full_system",
                "--debug_yellow_full_system",
                "--debug_simulator",
            ]
        ):
            print(
                "Do not run simulated pytests as a test when debugging, use run instead."
            )
            sys.exit(1)
        command += [f'--test_arg="{arg}"' for arg in runtime_args]
    else:
        command += runtime_args

    return command


def execute_command(command: list[str], print_only: bool = False):
    """Print or execute a Bazel command, exiting with its return code.

    :param command: the command tokens to join and run
    :param print_only: if True, only print the command without executing it
    """
    cmd_str = " ".join(command)
    if print_only:
        print(cmd_str)
    else:
        print(f"\n{'=' * 33} Running: {'=' * 38}\n\n{cmd_str}\n\n{'=' * 81}\n")
        code = os.system(cmd_str)
        sys.exit(1 if code != 0 else 0)


def start_interactive_cli():
    """Run the menu-driven interactive CLI.

    Walks the user through a series of questionary prompts to assemble a
    :class:`BuildConfig`, then validates, builds, and executes the resulting
    Bazel command. The menu choices (and their inline descriptions) live in
    cli_params.py. Returns early without running anything if the user aborts
    the top-level prompt.
    """
    config = BuildConfig(action=ActionArgument.run)  # Default action
    extra_args = []

    category = questionary.select(
        "What would you like to do?",
        choices=CATEGORY_CHOICES,
        style=INTERACTIVE_STYLE,
    ).ask()

    if not category:
        return

    match category:
        case "Run thunderscope":
            config.action = ActionArgument.run
            config.search_query = "thunderscope"
            launch = questionary.select(
                "Launch mode?",
                choices=LAUNCH_MODE_CHOICES,
                style=INTERACTIVE_STYLE,
            ).ask()
            if launch == "Simulator":
                selected = questionary.checkbox(
                    "Options:",
                    choices=THUNDERSCOPE_SIMULATOR_OPTION_CHOICES,
                    style=INTERACTIVE_STYLE,
                ).ask()
                for opt in selected:
                    extra_args.extend([f"--{opt}"])
                    if opt == "record_stats":
                        time = questionary.text(
                            "Enter record stats duration (minutes):",
                            style=INTERACTIVE_STYLE,
                        ).ask()
                        extra_args.extend([time])
            else:
                iface = questionary.text(
                    "Network interface?", style=INTERACTIVE_STYLE
                ).ask()
                extra_args.extend(["--run_diagnostics", "--interface", iface])

        case "Test":
            config.action = ActionArgument.test
            test_name = questionary.text(
                "Enter test name (leave empty for entire suite)",
                style=INTERACTIVE_STYLE,
            ).ask()
            if not test_name:
                config.test_suite = True
            else:
                config.search_query = test_name
                runs_str = questionary.text(
                    "Number of times to run each test (leave empty for 1):",
                    style=INTERACTIVE_STYLE,
                ).ask()
                if runs_str and runs_str.isdigit() and int(runs_str) > 1:
                    config.runs = int(runs_str)

        case "Flash":
            config.action = ActionArgument.run
            config.search_query = "ansible"
            playbook_choice = questionary.select(
                "Select playbook:",
                choices=PLAYBOOK_CHOICES,
                style=INTERACTIVE_STYLE,
            ).ask()
            # The DEBUG_POWERLOOP entry reuses the deploy_powerboard playbook but
            # compiles powerloop_main with the DEBUG_POWERLOOP flag, swapping in
            # the bare setup()/loop() stubs so arbitrary code can be flashed onto
            # the powerboard microcontroller for debugging.
            if playbook_choice == DEBUG_POWERLOOP_PLAYBOOK:
                config.ansible_playbook = "deploy_powerboard.yml"
                config.debug_powerloop = True
            else:
                config.ansible_playbook = playbook_choice
            config.robot_name = questionary.text(
                "Robot name?", style=INTERACTIVE_STYLE
            ).ask()
            config.ssh_password = questionary.password(
                "SSH password?", style=INTERACTIVE_STYLE
            ).ask()

    validate(config)
    command = create_command(config, extra_args)
    execute_command(command)


def fuzzy_find_target(
    action: ActionArgument, search_query: str, interactive_search: bool
) -> str:
    """Resolve a search query to a concrete Bazel target via fuzzy matching.

    Queries Bazel for the candidate targets relevant to the action (tests,
    binaries, and/or libraries) and fuzzy-matches the search query against
    their names. If interactive search is requested, or the best match falls
    below the confidence threshold, the user picks from the top matches via an
    fzf prompt; otherwise the best match is used directly.

    :param action: the Bazel action, which determines the candidate target kinds
    :param search_query: the query to match against target names
    :param interactive_search: force the interactive fzf picker
    :return: the fully-qualified Bazel target label
    """
    test_query = ["bazel", "query", "tests(//...)"]
    binary_query = ["bazel", "query", "kind(.*_binary,//...)"]
    library_query = ["bazel", "query", "kind(.*_library,//...)"]

    bazel_queries = {
        ActionArgument.test: [test_query],
        ActionArgument.run: [test_query, binary_query],
        ActionArgument.build: [library_query, test_query, binary_query],
    }

    targets = list(
        itertools.chain.from_iterable(
            run(q, stdout=PIPE).stdout.rstrip(b"\n").split(b"\n")
            for q in bazel_queries[action]
        )
    )
    target_dict = {target.split(b":")[-1]: target for target in targets}

    most_similar_target_name, confidence = process.extract(
        search_query, list(target_dict.keys()), limit=1
    )[0]
    target = str(target_dict[most_similar_target_name], encoding="utf-8")

    if interactive_search or confidence < THEFUZZ_MATCH_RATIO_THRESHOLD:
        filtered = process.extract(
            search_query, list(target_dict.keys()), limit=NUM_FILTERED_MATCHES_TO_SHOW
        )
        selected_name = iterfzf.iterfzf(iter([name for name, _ in filtered]))
        target = str(target_dict[selected_name.encode()], encoding="utf-8")
    else:
        print(f"Found target {target} (confidence {confidence})")

    return target


if __name__ == "__main__":
    app()
