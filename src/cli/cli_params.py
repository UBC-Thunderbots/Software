from enum import Enum
from typing import Annotated

import questionary
from typer import Argument, Option

from cli.multi_option import MultiOption


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
CATEGORY_CHOICES = [
    questionary.Choice(
        title="Run thunderscope",
        description="Launch Thunderscope against the simulator or real robots",
    ),
    questionary.Choice(
        title="Test",
        description="Run a single test or the entire test suite",
    ),
    questionary.Choice(
        title="Flash",
        description="Deploy software or firmware to a robot via Ansible",
    ),
]

# Thunderscope "Launch mode?" menu.
LAUNCH_MODE_CHOICES = [
    questionary.Choice(
        title="Simulator",
        description="Run Thunderscope against the simulated full system",
    ),
    questionary.Choice(
        title="Diagnostics",
        description="Run Thunderscope in diagnostics mode against real robots",
    ),
]

# Thunderscope simulator "Options:" checkbox.
THUNDERSCOPE_SIMULATOR_OPTION_CHOICES = [
    questionary.Choice(
        title="enable_autoref",
        description="Run the autoref alongside the simulator",
    ),
    questionary.Choice(
        title="ci_mode",
        description="Run in continuous integration mode (headless friendly)",
    ),
    questionary.Choice(
        title="record_stats",
        description="Record gameplay statistics for a given duration in minutes",
    ),
    questionary.Choice(
        title="enable_realism",
        description="Enable realistic simulation physics and sensor noise",
    ),
    questionary.Choice(
        title="enable_autogc",
        description="Run the automated game controller alongside the simulator",
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
        description="First-time setup of the Raspberry Pi on a robot",
    ),
    questionary.Choice(
        title="deploy_robot_software.yml",
        description="Build and flash Thunderloop and the robot software",
    ),
    questionary.Choice(
        title="deploy_powerboard.yml",
        description="Flash the powerboard firmware (powerloop_main)",
    ),
    questionary.Choice(
        title=DEBUG_POWERLOOP_PLAYBOOK,
        description="Flash powerloop_main built with the DEBUG_POWERLOOP flag "
        "for inserting arbitrary debug code onto the powerboard",
    ),
    questionary.Choice(
        title="deploy_motor_firmware.yml",
        description="Flash the motor controller firmware",
    ),
]
