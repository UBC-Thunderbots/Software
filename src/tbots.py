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
    disable_power_service: bool = False
    disable_motor_service: bool = False


class BazelFlag(tuple, Enum):
    DEBUG_BUILD = ("-c", "dbg")
    OPTIMIZED = ("--copt=-O3",)
    ROBOT_PLATFORM = ("--platforms=//toolchains/cc:robot",)
    TRACY = ("--cxxopt=-DTRACY_ENABLE",)
    THUNDERSCOPE = ("--spawn_strategy=local", "--test_env=DISPLAY=:0")
    NO_CACHE_TESTS = ("--cache_test_results=false",)
    DEBUG_POWERLOOP = ("--//software/power:debug_powerloop",)
    DISABLE_POWER_SERVICE = ("--//software/embedded:disable_power_service",)
    DISABLE_MOTOR_SERVICE = ("--//software/embedded:disable_motor_service",)


app = Typer()

class InteractiveCli:
    HISTORY_FILE = "/tmp/tbots_history"
    HISTORY_MAX_ENTRIES = 50

    # ---------------------------------------------------------------------------
    # Interactive CLI styling
    #
    # Shared questionary style applied to every interactive prompt. The highlighted
    # (pointed-at) option is rendered in bold cyan so it stands out, while the other
    # rows are dimmed. questionary draws the per-option description on a line at the
    # bottom of the prompt using the same "text" class as the unselected rows, so it
    # inherits the dim styling and is distinguished by its "Description:" label and
    # position. Colours use ANSI names so they adapt to the user's terminal theme.
    # ---------------------------------------------------------------------------
    INTERACTIVE_STYLE = questionary.Style(
        [
            ("qmark", "fg:ansicyan bold"),
            ("question", "bold"),
            ("pointer", "fg:ansicyan bold"),
            ("highlighted", "fg:ansicyan bold"),
            ("selected", "fg:ansigreen"),
            ("answer", "fg:ansicyan bold"),
            ("text", "fg:ansibrightblack"),
            ("instruction", "fg:ansibrightblack italic"),
            ("disabled", "fg:ansibrightblack italic"),
        ]
    )


    # ---------------------------------------------------------------------------
    # Interactive CLI choices
    #
    # Each questionary.Choice pairs an option's display title with a description.
    # questionary renders the description inline when the option is highlighted in
    # the interactive menus, giving users guidance without leaving the prompt.
    # When a Choice has no explicit value, questionary returns its title, so the
    # titles below double as the values consumed by start_interactive_cli.
    # ---------------------------------------------------------------------------

    # Top-level "What would you like to do?" menu.
    class Category(str, Enum):
        THUNDERSCOPE = "thunderscope"
        TEST = "test"
        FLASH = "flash"
        REPEAT_CMD_MSG = "Repeat a past command"
    CATEGORY_CHOICES = [
        questionary.Choice(
            title="Run thunderscope",
            value=Category.THUNDERSCOPE,
            description="Launch Thunderscope against the simulator or real robots",
        ),
        questionary.Choice(
            title="Test",
            value=Category.TEST,
            description="Run a single test or the entire test suite",
        ),
        questionary.Choice(
            title="Flash",
            value=Category.FLASH,
            description="Deploy software or firmware to a robot via Ansible",
        ),
    ]

    class LaunchMode(str, Enum):
        SIM = "sim" # This value is not used, only the enum is checked
        RUN_DIAG = "run_diagnostics"
        RUN_BLUE = "run_blue"
        RUN_YELLOW = "run_yellow"
    # Thunderscope "Launch mode?" menu.
    LAUNCH_MODE_CHOICES = [
        questionary.Choice(
            title="Simulator",
            value = LaunchMode.SIM,
            description="Run Thunderscope against the simulated full system",
        ),
        questionary.Choice(
            title="Diagnostics",
            value=LaunchMode.RUN_DIAG,
            description="Run Thunderscope in diagnostics mode against real robots",
        ),
        questionary.Choice(
            title="Run blue",
            value=LaunchMode.RUN_BLUE,
            description="run_blue",
        ),
        questionary.Choice(
            title="Run yellow",
            value=LaunchMode.RUN_YELLOW,
            description="run_yellow",
        ),
    ]

    class SimOptions(str, Enum):
        RECORD_STATS = "record_stats"
        ENABLE_AUTOREF = "enable_autoref"
        CI_MODE = "ci_mode"
        ENABLE_REALISM = "enable_realism"
        ENABLE_AUTOGC = "enable_autogc"
    # Thunderscope simulator "Options:" checkbox.
    THUNDERSCOPE_SIMULATOR_OPTION_CHOICES = [
        questionary.Choice(
            title="Enable Automatic Referee",
            value=SimOptions.ENABLE_AUTOREF,
            description="Run the autoref alongside the simulator",
        ),
        questionary.Choice(
            title="CI Mode",
            value=SimOptions.CI_MODE,
            description="Run in continuous integration mode (headless friendly)",
        ),
        questionary.Choice(
            title="Record Statistics",
            value=SimOptions.RECORD_STATS,
            description="Record gameplay statistics for a given duration in minutes",
        ),
        questionary.Choice(
            title="Enable Realism",
            value=SimOptions.ENABLE_REALISM,
            description="Enable realistic simulation physics and sensor noise",
        ),
        questionary.Choice(
            title="Enable Automatic Game Controller",
            value=SimOptions.ENABLE_AUTOGC,
            description="Run the automated game controller alongside the simulator",
        ),
    ]

    # Deploy robot software "Options:" checkbox. Each option compiles Thunderloop
    # with a preprocessor flag that disables the corresponding service, letting
    # Thunderloop run on a robot whose powerboard or motorboard is unavailable.
    class DeployOption(str, Enum):
        DISABLE_POWER_SERVICE = "DISABLE_POWER_SERVICE"
        DISABLE_MOTOR_SERVICE = "DISABLE_MOTOR_SERVICE"
    DEPLOY_ROBOT_SOFTWARE_OPTION_CHOICES = [
        questionary.Choice(
            title="Disable Power Service",
            value=DeployOption.DISABLE_POWER_SERVICE,
            description="Compile Thunderloop without the Power Service (no powerboard)",
        ),
        questionary.Choice(
            title="Disable Motor Service",
            value=DeployOption.DISABLE_MOTOR_SERVICE,
            description="Compile Thunderloop without the Motor Service (no motorboard)",
        ),
    ]

    # Marker value returned by the DEBUG_POWERLOOP playbook choice. It maps to the
    # deploy_powerboard.yml playbook but additionally compiles powerloop_main with
    # the DEBUG_POWERLOOP flag, swapping in bare setup()/loop() stubs so arbitrary
    # code can be flashed onto the powerboard microcontroller for debugging.
    DEBUG_POWERLOOP_PLAYBOOK = "deploy_powerboard.yml (DEBUG_POWERLOOP)"

    # Flash "Select playbook:" menu.
    PLAYBOOK_CHOICES = [
        questionary.Choice(
            title="setup_pi.yml",
            value=("setup_pi.yml", False),
            description="First-time setup of the Raspberry Pi on a robot",
        ),
        # If the playbook name changes then you must change the if statement below.
        questionary.Choice(
            title="deploy_robot_software.yml",
            value=("deploy_robot_software.yml", False),
            description="Build and flash Thunderloop and the robot software",
        ),
        questionary.Choice(
            title="deploy_powerboard.yml",
            value=("deploy_powerboard.yml", False),
            description="Flash the powerboard firmware (powerloop_main)",
        ),
        questionary.Choice(
            title=DEBUG_POWERLOOP_PLAYBOOK,
            value=("deploy_powerboard.yml", True),
            description="Flash powerloop_main built with the DEBUG_POWERLOOP flag "
                        "for inserting arbitrary debug code onto the powerboard",
        ),
        questionary.Choice(
            title="deploy_motor_firmware.yml",
            value=("deploy_motor_firmware.yml", False),
            description="Flash the motor controller firmware",
        ),
    ]


    @staticmethod
    def load_history() -> list[str]:
        if not os.path.exists(InteractiveCli.HISTORY_FILE):
            return []
        with open(InteractiveCli.HISTORY_FILE) as f:
            lines = [line.strip() for line in f.readlines()]
        return [l for l in lines if l]

    @staticmethod
    def save_to_history(cmd_str: str):
        history = InteractiveCli.load_history()
        history = [h for h in history if h != cmd_str]
        history.append(cmd_str)
        history = history[-InteractiveCli.HISTORY_MAX_ENTRIES:]
        with open(InteractiveCli.HISTORY_FILE, "w") as f:
            f.write("\n".join(history) + "\n")

    @staticmethod
    def start_interactive_cli():
        """Run the menu-driven interactive CLI.

        Walks the user through a series of questionary prompts to assemble a
        :class:`BuildConfig`, then validates, builds, and executes the resulting
        Bazel command. Returns early without running anything if the user aborts
        the top-level prompt.
        """
        config = BuildConfig(action=ActionArgument.run)  # Default action
        extra_args = []

        history = InteractiveCli.load_history()
        choices = InteractiveCli.CATEGORY_CHOICES
        if history:
            choices = [InteractiveCli.Category.REPEAT_CMD_MSG.value] + InteractiveCli.CATEGORY_CHOICES

        category = questionary.select(
            "What would you like to do?",
            choices=choices,
            style=InteractiveCli.INTERACTIVE_STYLE,
        ).unsafe_ask()

        match category:
            case InteractiveCli.Category.REPEAT_CMD_MSG:
                past_cmd = questionary.select(
                    "Select a command to re-run:",
                    choices=list(reversed(history)),
                ).unsafe_ask()
                if not past_cmd:
                    return
                print(f"\n{'=' * 33} Running: {'=' * 38}\n\n{past_cmd}\n\n{'=' * 81}\n")
                InteractiveCli.save_to_history(past_cmd)
                code = os.system(past_cmd)
                sys.exit(1 if code != 0 else 0)
            case InteractiveCli.Category.THUNDERSCOPE:
                config.action = ActionArgument.run
                config.search_query = "thunderscope"
                launch = questionary.select(
                    "Launch mode?",
                    choices=InteractiveCli.LAUNCH_MODE_CHOICES,
                    style=InteractiveCli.INTERACTIVE_STYLE,
                ).unsafe_ask()
                if launch == InteractiveCli.LaunchMode.SIM:
                    selected = questionary.checkbox(
                        "Options:",
                        choices=InteractiveCli.THUNDERSCOPE_SIMULATOR_OPTION_CHOICES,
                        style=InteractiveCli.INTERACTIVE_STYLE,
                    ).unsafe_ask()
                    for opt in selected:
                        extra_args.extend([f"--{opt.value}"])
                        if opt == InteractiveCli.SimOptions.RECORD_STATS:
                            duration = questionary.text(
                                "Enter record stats duration (minutes):",
                                style=InteractiveCli.INTERACTIVE_STYLE,
                                validate= lambda x: x.isdigit(),
                            ).unsafe_ask()
                            extra_args.extend([duration])
                else:
                    iface = questionary.text(
                        "Network interface?", style=InteractiveCli.INTERACTIVE_STYLE
                    ).unsafe_ask()
                    extra_args.extend(
                        [f"--{launch.value}", "--interface", iface]
                    )

            case InteractiveCli.Category.TEST:
                config.action = ActionArgument.test
                test_name = questionary.text(
                    "Enter test name (leave empty for entire suite)",
                    style=InteractiveCli.INTERACTIVE_STYLE,
                ).unsafe_ask()
                if not test_name:
                    config.test_suite = True
                else:
                    config.search_query = test_name
                    runs_str = questionary.text(
                        "Number of times to run each test (leave empty for 1):",
                        style=InteractiveCli.INTERACTIVE_STYLE,
                    ).unsafe_ask()
                    if runs_str and runs_str.isdigit() and int(runs_str) > 1:
                        config.runs = int(runs_str)

            case InteractiveCli.Category.FLASH:
                config.action = ActionArgument.run
                config.search_query = "ansible"
                playbook, debug_powerloop = questionary.select(
                    "Select playbook:",
                    choices=InteractiveCli.PLAYBOOK_CHOICES,
                    style=InteractiveCli.INTERACTIVE_STYLE,
                ).unsafe_ask()

                config.ansible_playbook = playbook
                config.debug_powerloop = debug_powerloop

                if config.ansible_playbook == "deploy_robot_software.yml":
                    selected = (
                            questionary.checkbox(
                                "Options:",
                                choices=InteractiveCli.DEPLOY_ROBOT_SOFTWARE_OPTION_CHOICES,
                                style=InteractiveCli.INTERACTIVE_STYLE,
                            ).unsafe_ask()
                            or []
                    )
                    config.disable_power_service = InteractiveCli.DeployOption.DISABLE_POWER_SERVICE in selected
                    config.disable_motor_service = InteractiveCli.DeployOption.DISABLE_MOTOR_SERVICE in selected
                config.robot_name = questionary.text(
                    "Robot name?", style=InteractiveCli.INTERACTIVE_STYLE
                ).unsafe_ask()
                config.ssh_password = questionary.password(
                    "SSH password?", style=InteractiveCli.INTERACTIVE_STYLE
                ).unsafe_ask()

        validate(config)
        command = create_command(config, extra_args)
        execute_command(command)


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
        InteractiveCli.start_interactive_cli()
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
                      -//toolchains/...                   \\
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
        BazelFlag.DISABLE_POWER_SERVICE: config.disable_power_service,
        BazelFlag.DISABLE_MOTOR_SERVICE: config.disable_motor_service,
    }
    for flag, condition in flag_conditions.items():
        if condition:
            command += list(flag.value)

    if config.test_suite and config.action == ActionArgument.test:
        command += ["--build_tests_only"]

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
        InteractiveCli.save_to_history(cmd_str)
        code = os.system(cmd_str)
        sys.exit(1 if code != 0 else 0)




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
