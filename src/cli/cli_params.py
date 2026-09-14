import os
import sys
from dataclasses import dataclass
from enum import Enum
from typing import Annotated

import questionary
from cli.multi_option import MultiOption
from typer import Argument, Option


class ActionArgument(str, Enum):
    build = "build"
    test = "test"
    run = "run"


class DebugBinary(str, Enum):
    sim = "sim"
    blue = "blue"
    yellow = "yellow"


SearchQueryArgument = Annotated[
    str | None, Argument(help="Search query for bazel target")
]

PrintCommandOption: type[bool] = Annotated[
    bool, Option("-p", "--print_command", help="Print the generated Bazel command")
]

NoOptimizedBuildOption: type[bool] = Annotated[
    bool,
    Option(
        "-no", "--no_optimized_build", help="Compile binaries without -O3 optimizations"
    ),
]

DebugBuildOption: type[bool] = Annotated[
    bool,
    Option(
        "-d",
        "--debug",
        help="Compile binaries with debug symbols",
    ),
]

SelectDebugBinariesOption = Annotated[
    list[DebugBinary] | None,
    MultiOption(
        "-ds",
        "--select_debug_binaries",
        help="Select all binaries which are running separately in debug mode",
    ),
]

FlashRobotsOption = Annotated[
    list[int] | None,
    MultiOption(
        "-f",
        "--flash_robots",
        help="A list of space separated integers representing the robot IDs "
        "that should be flashed by the deploy_robot_software Ansible playbook",
    ),
]

SSHPasswordOption = Annotated[
    str,
    Option(
        "-pwd", "--pwd", help="Password used by Ansible when SSHing into the robots"
    ),
]

InteractiveModeOption = Annotated[
    bool,
    Option(
        "-i",
        "--interactive",
        help="Enables interactive searching for bazel targets",
    ),
]

TracyOption = Annotated[
    bool, Option("--tracy", help="Run the binary with the TRACY_ENABLE macro defined")
]

TestSuiteOption = Annotated[
    bool,
    Option("--suite", help="Run entire test suite instead of searching for a target"),
]

EnableThunderscopeOption = Annotated[bool, Option("-t", "--enable_thunderscope")]
StopAIOnStartOption = Annotated[bool, Option("-s", "--stop_ai_on_start")]

JobsOption = Annotated[str, Option("-j", "--jobs")]
RunsOption = Annotated[
    int,
    Option(
        "-r", "--runs", help="Number of times to run each test (bazel --runs_per_test)"
    ),
]
RobotName = Annotated[
    str, Option("-rn", "--robot_name", help="Name of robot. E.g. balle")
]
AnsiblePlaybook = Annotated[
    str, Option("-ap", "--ansible_playbook", help="Ansible playbook name")
]


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


class InteractiveCli:
    HISTORY_FILE = "/tmp/tbots_history"
    HISTORY_DELIMITER = "<<||>>"
    HISTORY_MAX_ENTRIES = 50

    # ---------------------------------------------------------------------------
    # Interactive CLI styling
    #
    # Shared questionary style applied to every interactive prompt.
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
    # All values below encode a label for history tracking and a value for argument
    # creation.
    # ---------------------------------------------------------------------------

    # Top-level "What would you like to do?" menu.
    class Category(str, Enum):
        THUNDERSCOPE = "thunderscope"
        TEST = "test"
        FLASH = "flash"
        REPEAT = "repeat"  # this value is not used, only the enum is checked

    CATEGORY_CHOICES = [
        questionary.Choice(
            title="Repeat a past command",
            value=("", Category.REPEAT),
            description="Show a list of previous commands to select from",
        ),
        questionary.Choice(
            title="Run Thunderscope",
            value=("Run Thunderscope ", Category.THUNDERSCOPE),
            description="Launch Thunderscope against the simulator or real robots",
        ),
        questionary.Choice(
            title="Test",
            value=("Test ", Category.TEST),
            description="Run a single test or the entire test suite",
        ),
        questionary.Choice(
            title="Flash",
            value=("Flash ", Category.FLASH),
            description="Deploy software or firmware to a robot via Ansible",
        ),
    ]

    class LaunchMode(str, Enum):
        SIM = "sim"  # This value is not used, only the enum is checked
        RUN_DIAG = "run_diagnostics"
        RUN_BLUE = "run_blue"
        RUN_YELLOW = "run_yellow"

    # Thunderscope "Launch mode?" menu.
    LAUNCH_MODE_CHOICES = [
        questionary.Choice(
            title="Simulator",
            value=("as simulator ", LaunchMode.SIM),
            description="Run Thunderscope against the simulated full system",
        ),
        questionary.Choice(
            title="Diagnostics",
            value=("in diagnostic mode ", LaunchMode.RUN_DIAG),
            description="Run Thunderscope in diagnostics mode against real robots",
        ),
        questionary.Choice(
            title="Run blue",
            value=("as blue team", LaunchMode.RUN_BLUE),
            description="Run as blue team",
        ),
        questionary.Choice(
            title="Run yellow",
            value=("as yellow team", LaunchMode.RUN_YELLOW),
            description="Run as yellow team",
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
            value=("with autoref, ", SimOptions.ENABLE_AUTOREF),
            description="Run the autoref alongside the simulator",
        ),
        questionary.Choice(
            title="CI Mode",
            value=("in CI, ", SimOptions.CI_MODE),
            description="Run in continuous integration mode (headless friendly)",
        ),
        questionary.Choice(
            title="Record Statistics",
            value=("recording stats, ", SimOptions.RECORD_STATS),
            description="Record gameplay statistics for a given duration in minutes",
        ),
        questionary.Choice(
            title="Enable Realism",
            value=("with realism, ", SimOptions.ENABLE_REALISM),
            description="Enable realistic simulation physics and sensor noise",
        ),
        questionary.Choice(
            title="Enable Automatic Game Controller",
            value=("with gc, ", SimOptions.ENABLE_AUTOGC),
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
            value=("without power service, ", DeployOption.DISABLE_POWER_SERVICE),
            description="Compile Thunderloop without the Power Service (no powerboard)",
        ),
        questionary.Choice(
            title="Disable Motor Service",
            value=("without motor service, ", DeployOption.DISABLE_MOTOR_SERVICE),
            description="Compile Thunderloop without the Motor Service (no motorboard)",
        ),
    ]

    ROBOT_SOFTWARE_PLAYBOOK_NAME = "deploy_robot_software.yml"
    # Flash "Select playbook:" menu.
    PLAYBOOK_CHOICES = [
        questionary.Choice(
            title="setup_pi.yml",
            value=("PI setup ", "setup_pi.yml", False),
            description="First-time setup of the Raspberry Pi on a robot",
        ),
        questionary.Choice(
            title=ROBOT_SOFTWARE_PLAYBOOK_NAME,
            value=("robot software ", ROBOT_SOFTWARE_PLAYBOOK_NAME, False),
            description="Build and flash Thunderloop and the robot software",
        ),
        questionary.Choice(
            title="deploy_powerboard.yml",
            value=("powerboard firmware ", "deploy_powerboard.yml", False),
            description="Flash the powerboard firmware (powerloop_main)",
        ),
        # Maps to the deploy_powerboard.yml playbook but additionally compiles
        # powerloop_main with the DEBUG_POWERLOOP flag, swapping in bare setup()/loop()
        # stubs so arbitrary code can be flashed onto the powerboard microcontroller for debugging.
        questionary.Choice(
            title="deploy_powerboard.yml (DEBUG_POWERLOOP)",
            value=("powerboard firmware with debug ", "deploy_powerboard.yml", True),
            description="Flash powerloop_main built with the DEBUG_POWERLOOP flag "
            "for inserting arbitrary debug code onto the powerboard",
        ),
        questionary.Choice(
            title="deploy_motor_firmware.yml",
            value=("motorboard firmware ", "deploy_motor_firmware.yml", False),
            description="Flash the motor controller firmware",
        ),
    ]

    @staticmethod
    def load_history() -> list[str]:
        if not os.path.exists(InteractiveCli.HISTORY_FILE):
            return []
        with open(InteractiveCli.HISTORY_FILE) as f:
            lines = [line.strip() for line in f.readlines()]
        return [line.replace("\\n", "\n") for line in lines if line]

    @staticmethod
    def save_to_history(cmd_title: str, cmd_str: str):
        history = InteractiveCli.load_history()
        history = [h for h in history if h != cmd_str]
        history.append(cmd_title + InteractiveCli.HISTORY_DELIMITER + cmd_str)
        history = history[-InteractiveCli.HISTORY_MAX_ENTRIES :]
        with open(InteractiveCli.HISTORY_FILE, "w") as f:
            f.write("\n".join(h.replace("\n", "\\n") for h in history) + "\n")

    @staticmethod
    def start_interactive_cli(
        config: BuildConfig,
    ) -> tuple[str, BuildConfig, list[str]] | None:
        """Run the menu-driven interactive CLI.

        Walks the user through a series of questionary prompts to assemble a
        :class:`BuildConfig` by modifying an existing config. Returns early without
        running anything if the user aborts the top-level prompt.

        :param config: The cli config to modify.

        :returns The build title, build config assembled from the user prompts, and
        extra args. None if failure to build config.
        """
        config.action = ActionArgument.run  # Default action
        extra_args = []

        history = InteractiveCli.load_history()
        choices = InteractiveCli.CATEGORY_CHOICES[1:]
        if history:
            choices = InteractiveCli.CATEGORY_CHOICES[0:]

        category_label, category = questionary.select(
            "What would you like to do?",
            choices=choices,
            style=InteractiveCli.INTERACTIVE_STYLE,
        ).unsafe_ask()

        cmd_title = category_label

        match category:
            case InteractiveCli.Category.REPEAT:
                history_choices = [
                    questionary.Choice(
                        title=entry.split(InteractiveCli.HISTORY_DELIMITER)[0],
                        value=entry.split(InteractiveCli.HISTORY_DELIMITER)[1],
                        description=entry.split(InteractiveCli.HISTORY_DELIMITER)[1],
                    )
                    for entry in list(reversed(history))
                ]
                past_cmd = questionary.select(
                    "Select a command to re-run:",
                    choices=history_choices,
                ).unsafe_ask()

                if not past_cmd:
                    return
                print(f"\n{'=' * 33} Running: {'=' * 38}\n\n{past_cmd}\n\n{'=' * 81}\n")
                code = os.system(past_cmd)
                sys.exit(1 if code != 0 else 0)

            case InteractiveCli.Category.THUNDERSCOPE:
                config.action = ActionArgument.run
                config.search_query = "thunderscope"

                launch_label, launch_mode = questionary.select(
                    "Launch mode?",
                    choices=InteractiveCli.LAUNCH_MODE_CHOICES,
                    style=InteractiveCli.INTERACTIVE_STYLE,
                ).unsafe_ask()
                cmd_title += launch_label

                if launch_mode == InteractiveCli.LaunchMode.SIM:
                    sim_option_selection = questionary.checkbox(
                        "Options:",
                        choices=InteractiveCli.THUNDERSCOPE_SIMULATOR_OPTION_CHOICES,
                        style=InteractiveCli.INTERACTIVE_STYLE,
                    ).unsafe_ask()
                    if sim_option_selection:
                        sim_option_labels, sim_options = zip(*sim_option_selection)
                        for label in sim_option_labels:
                            cmd_title += label

                        for opt in sim_options:
                            extra_args.extend([f"--{opt.value}"])
                            if opt == InteractiveCli.SimOptions.RECORD_STATS:
                                duration = questionary.text(
                                    "Enter record stats duration (minutes):",
                                    style=InteractiveCli.INTERACTIVE_STYLE,
                                    validate=lambda x: x.isdigit(),
                                ).unsafe_ask()
                                extra_args.extend([duration])
                else:
                    iface = questionary.text(
                        "Network interface?", style=InteractiveCli.INTERACTIVE_STYLE
                    ).unsafe_ask()
                    extra_args.extend([f"--{launch_mode.value}", "--interface", iface])

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
                flash_cmd_label, playbook, debug_powerloop = questionary.select(
                    "Select playbook:",
                    choices=InteractiveCli.PLAYBOOK_CHOICES,
                    style=InteractiveCli.INTERACTIVE_STYLE,
                ).unsafe_ask()
                cmd_title += flash_cmd_label

                config.ansible_playbook = playbook
                config.debug_powerloop = debug_powerloop

                if (
                    config.ansible_playbook
                    == InteractiveCli.ROBOT_SOFTWARE_PLAYBOOK_NAME
                ):
                    robot_software_config = (
                        questionary.checkbox(
                            "Options:",
                            choices=InteractiveCli.DEPLOY_ROBOT_SOFTWARE_OPTION_CHOICES,
                            style=InteractiveCli.INTERACTIVE_STYLE,
                        ).unsafe_ask()
                        or []
                    )
                    if robot_software_config:
                        robot_software_option_labels, robot_software_options = zip(
                            *robot_software_config
                        )
                        for label in robot_software_option_labels:
                            cmd_title += label
                        config.disable_power_service = (
                            InteractiveCli.DeployOption.DISABLE_POWER_SERVICE
                            in robot_software_options
                        )
                        config.disable_motor_service = (
                            InteractiveCli.DeployOption.DISABLE_MOTOR_SERVICE
                            in robot_software_options
                        )

                name = questionary.text(
                    "Robot name?", style=InteractiveCli.INTERACTIVE_STYLE
                ).unsafe_ask()
                config.robot_name = name
                cmd_title += "to " + name
                config.ssh_password = questionary.password(
                    "SSH password?", style=InteractiveCli.INTERACTIVE_STYLE
                ).unsafe_ask()

        return cmd_title, config, extra_args
