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
    InteractiveModeOption,
    JobsOption,
    NoOptimizedBuildOption,
    PrintCommandOption,
    RobotName,
    SelectDebugBinariesOption,
    SSHPasswordOption,
    StopAIOnStartOption,
    TestSuiteOption,
    TracyOption,
)

THEFUZZ_MATCH_RATIO_THRESHOLD = 50
NUM_FILTERED_MATCHES_TO_SHOW = 10

@dataclass
class BuildOptions:
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
    enable_visualizer: bool = False
    stop_ai_on_start: bool = False
    jobs_option: str | None = None
    robot_name: str | None = None
    ansible_playbook: str | None = None

class BazelFlag(tuple, Enum):
    DEBUG_BUILD = ("-c", "dbg")
    OPTIMIZED = ("--copt=-O3",)
    ROBOT_PLATFORM = ("--platforms=//toolchains/cc:robot",)
    TRACY = ("--cxxopt=-DTRACY_ENABLE",)
    THUNDERSCOPE = ("--spawn_strategy=local", "--test_env=DISPLAY=:0")
    NO_CACHE_TESTS = ("--cache_test_results=false",)

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
    enable_visualizer: EnableVisualizerOption = False,
    stop_ai_on_start: StopAIOnStartOption = False,
    jobs_option: JobsOption = None,
    robot_name: RobotName = None,
    ansible_playbook: AnsiblePlaybook = None,
) -> None:
    if not action and not search_query:
        start_interactive_cli()
        return

    opts = BuildOptions(
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
        enable_visualizer=enable_visualizer,
        stop_ai_on_start=stop_ai_on_start,
        jobs_option=jobs_option,
        robot_name=robot_name,
        ansible_playbook=ansible_playbook,
    )

    validate(opts)
    command = create_command(opts, ctx.args)
    execute_command(command, print_only=print_command)


def validate(opts: BuildOptions):
    if bool(opts.flash_robots) or bool(opts.ansible_playbook):
        if not opts.ssh_password:
            print("Error: SSH password is required for flashing or ansible playbooks.")
            sys.exit(1)
    if opts.search_query is None and (not opts.test_suite or opts.action != ActionArgument.test):
        print("Error: Specify a search query or use --suite with test.")
        sys.exit(1)


def create_command(opts: BuildOptions, extra_args: list[str]) -> list[str]:
    """Builds the bazel command list based on options and pass-through args."""
    if opts.test_suite and opts.action == ActionArgument.test:
        target = """-- //...                              \\
                      -//software/field_tests/...         \\
                      -//toolchains/cc/...                \\
                      -//software:unix_full_system_tar_gen"""
    else:
        target = fuzzy_find_target(opts.action, opts.search_query, opts.interactive_search)

    command = ["bazel", opts.action.value]
    runtime_args = list(extra_args)

    # Apply Bazel Flags
    flag_conditions = {
        BazelFlag.DEBUG_BUILD: opts.debug_build or bool(opts.select_debug_binaries),
        BazelFlag.OPTIMIZED: not opts.debug_build and (not opts.no_optimized_build or bool(opts.flash_robots)),
        BazelFlag.ROBOT_PLATFORM: bool(opts.flash_robots or opts.ansible_playbook),
        BazelFlag.TRACY: opts.tracy,
        BazelFlag.THUNDERSCOPE: opts.enable_thunderscope,
        BazelFlag.NO_CACHE_TESTS: opts.action == ActionArgument.test,
    }
    for flag, condition in flag_conditions.items():
        if condition:
            command += list(flag.value)

    if opts.jobs_option:
        command += [f"--jobs={opts.jobs_option}"]

    # Handle binary debugging flags
    if opts.select_debug_binaries:
        if DebugBinary.sim in opts.select_debug_binaries:
            runtime_args.append("--debug_simulator")
        if DebugBinary.blue in opts.select_debug_binaries:
            runtime_args.append("--debug_blue_full_system")
        if DebugBinary.yellow in opts.select_debug_binaries:
            runtime_args.append("--debug_yellow_full_system")

    command += [target]

    # Separator for runtime arguments
    if opts.action == ActionArgument.run:
        command += ["--"]

    # Append runtime arguments
    if opts.stop_ai_on_start: runtime_args.append("--stop_ai_on_start")
    if opts.enable_visualizer: runtime_args.append("--enable_visualizer")
    if opts.enable_thunderscope: runtime_args.append("--enable_thunderscope")

    if opts.ansible_playbook:
        runtime_args += ["--playbook", opts.ansible_playbook, "--hosts", f"{opts.robot_name}.local", "-pwd", opts.ssh_password]

    if opts.flash_robots:
        runtime_args += ["--playbook", "deploy_robot_software.yml", "--hosts"]
        runtime_args += [f"192.168.6.{200 + int(id)}" for id in opts.flash_robots]
        runtime_args += ["-pwd", opts.ssh_password]

    if opts.action == ActionArgument.test:
        # Safety check for pytest debugging
        if any(x in runtime_args for x in ["--debug_blue_full_system", "--debug_yellow_full_system", "--debug_simulator"]):
            print("Do not run simulated pytests as a test when debugging, use run instead.")
            sys.exit(1)
        command += [f'--test_arg="{arg}"' for arg in runtime_args]
    else:
        command += runtime_args

    return command


def execute_command(command: list[str], print_only: bool = False):
    cmd_str = " ".join(command)
    if print_only:
        print(cmd_str)
    else:
        print(f"\n{'='*33} Running: {'='*38}\n\n{cmd_str}\n\n{'='*81}\n")
        code = os.system(cmd_str)
        sys.exit(1 if code != 0 else 0)


def start_interactive_cli():
    """Interactive mode that builds BuildOptions and calls execution directly."""
    opts = BuildOptions()
    extra_args = []

    category = questionary.select(
        "What would you like to do?", 
        choices=["Run thunderscope", "Test", "Flash"]
    ).ask()

    if not category: return

    match category:
        case "Run thunderscope":
            opts.action = ActionArgument.run
            opts.search_query = "thunderscope"
            launch = questionary.select("Launch mode?", choices=["Simulator", "Diagnostics"]).ask()
            if launch == "Simulator":
                opts.enable_thunderscope = True
                selected = questionary.checkbox("Options:", 
                    choices=["enable_autoref", "ci_mode", "record_stats", "enable_realism", "show_autoref_gui"]).ask()
                extra_args.extend([f"--{opt}" for opt in selected])
            else:
                iface = questionary.text("Network interface?").ask()
                extra_args.extend(["--run_diagnostics", "--interface", iface])

        case "Test":
            opts.action = ActionArgument.test
            test_name = questionary.text("Enter test name (leave empty for entire suite)").ask()
            if not test_name:
                opts.test_suite = True
            else:
                opts.search_query = test_name

        case "Flash":
            opts.action = ActionArgument.run
            opts.search_query = "ansible"
            opts.ansible_playbook = questionary.select("Select playbook:", 
                choices=["setup_pi.yml", "deploy_robot_software.yml", "deploy_powerboard.yml"]).ask()
            opts.robot_name = questionary.text("Robot name?").ask()
            opts.ssh_password = questionary.password("SSH password?").ask()

    validate(opts)
    command = create_command(opts, extra_args)
    execute_command(command)


def fuzzy_find_target(action: ActionArgument, search_query: str, interactive_search: bool) -> str:
    test_query = ["bazel", "query", "tests(//...)"]
    binary_query = ["bazel", "query", "kind(.*_binary,//...)"]
    library_query = ["bazel", "query", "kind(.*_library,//...)"]
    
    bazel_queries = {
        ActionArgument.test: [test_query],
        ActionArgument.run: [test_query, binary_query],
        ActionArgument.build: [library_query, test_query, binary_query],
    }

    targets = list(itertools.chain.from_iterable(
        run(q, stdout=PIPE).stdout.rstrip(b"\n").split(b"\n") for q in bazel_queries[action]
    ))
    target_dict = {target.split(b":")[-1]: target for target in targets}

    most_similar_target_name, confidence = process.extract(
        search_query, list(target_dict.keys()), limit=1
    )[0]
    target = str(target_dict[most_similar_target_name], encoding="utf-8")

    if interactive_search or confidence < THEFUZZ_MATCH_RATIO_THRESHOLD:
        filtered = process.extract(search_query, list(target_dict.keys()), limit=NUM_FILTERED_MATCHES_TO_SHOW)
        selected_name = iterfzf.iterfzf(iter([name for name, _ in filtered]))
        target = str(target_dict[selected_name.encode()], encoding="utf-8")
    else:
        print(f"Found target {target} (confidence {confidence})")

    return target


if __name__ == "__main__":
    app()