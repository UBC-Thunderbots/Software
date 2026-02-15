from typer import Option, Argument
from enum import Enum
from typing import Annotated

from cli.multi_option import MultiOption


class ActionArgument(str, Enum):
    build = "build"
    test = "test"
    run = "run"


class DebugBinary(str, Enum):
    sim = "sim"
    blue = "blue"
    yellow = "yellow"


class Platform(str, Enum):
    PI = "PI"
    NANO = "NANO"
    LIMITED = "LIMITED"


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

PlatformOption = Annotated[
    Platform, Option("-pl", "--platform", help="The platform to build Thunderloop for")
]

TestSuiteOption = Annotated[
    bool,
    Option(
        "-s", "--suite", help="Run entire test suite instead of searching for a target"
    ),
]

EnableThunderscopeOption = Annotated[bool, Option("-t", "--enable_thunderscope")]
EnableVisualizerOption = Annotated[bool, Option("-v", "--enable_visualizer")]
StopAIOnStartOption = Annotated[bool, Option("--stop_ai_on_start")]
