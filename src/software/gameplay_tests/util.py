import argparse
import os
import sys

import pytest


def load_command_line_arguments(allow_unrecognized: bool = False):
    """Load in command-line arguments using argparse

    NOTE: Pytest has its own built in argument parser (conftest.py, pytest_addoption)
    but it doesn't seem to play nicely with bazel. We just use argparse instead.

    :param allow_unrecognized: if true, does not raise an error for unrecognized arguments
    """
    parser = argparse.ArgumentParser(
        description="Run simulated or field gameplay tests"
    )

    general_group = parser.add_argument_group("General test arguments")
    general_group.add_argument(
        "--run_field_test",
        action="store_true",
        default=False,
        help="Runs test as a field test instead of a simulated test",
    )
    general_group.add_argument(
        "--enable_thunderscope", action="store_true", help="enable thunderscope"
    )
    general_group.add_argument(
        "--test_filter",
        action="store",
        default="",
        help="The test filter, if not specified all tests will run. "
        + "See https://docs.pytest.org/en/latest/how-to/usage.html#specifying-tests-selecting-tests",
    )
    general_group.add_argument(
        "--blue_full_system_runtime_dir",
        type=str,
        help="blue full_system runtime directory",
        default="/tmp/tbots/blue",
    )
    general_group.add_argument(
        "--yellow_full_system_runtime_dir",
        type=str,
        help="yellow full_system runtime directory",
        default="/tmp/tbots/yellow",
    )
    general_group.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )
    general_group.add_argument(
        "--debug_blue_full_system",
        action="store_true",
        default=False,
        help="Debug blue full_system",
    )
    general_group.add_argument(
        "--debug_yellow_full_system",
        action="store_true",
        default=False,
        help="Debug yellow full_system",
    )
    general_group.add_argument(
        "--show_gamecontroller_logs",
        action="store_true",
        default=False,
        help="Show gamecontroller logs",
    )

    simulated_group = parser.add_argument_group("Simulated test arguments")
    simulated_group.add_argument(
        "--aggregate", action="store_true", default=False, help="Run aggregate test"
    )
    simulated_group.add_argument(
        "--simulator_runtime_dir",
        type=str,
        help="simulator runtime directory",
        default="/tmp/tbots",
    )
    simulated_group.add_argument(
        "--debug_simulator",
        action="store_true",
        default=False,
        help="Debug the simulator",
    )
    simulated_group.add_argument(
        "--enable_realism",
        action="store_true",
        default=False,
        help="Use realism in the simulator",
    )

    field_group = parser.add_argument_group("Field test arguments")
    field_group.add_argument(
        "--interface",
        action="store",
        type=str,
        default=None,
        help="Which interface to communicate over",
    )
    field_group.add_argument(
        "--channel",
        action="store",
        type=int,
        default=0,
        help="Which channel to communicate over",
    )
    field_group.add_argument(
        "--estop_baudrate",
        action="store",
        type=int,
        default=115200,
        help="Estop Baudrate",
    )
    field_group.add_argument(
        "--run_yellow",
        action="store_true",
        default=False,
        help="Run the test with friendly robots in yellow mode",
    )

    estop_group = field_group.add_mutually_exclusive_group()
    estop_group.add_argument(
        "--keyboard_estop",
        action="store_true",
        default=False,
        help="Allows the use of the spacebar as an estop instead of a physical one",
    )
    estop_group.add_argument(
        "--disable_communication",
        action="store_true",
        default=False,
        help="Disables checking for estop plugged in (ONLY USE FOR LOCAL TESTING)",
    )

    return parser.parse_known_args()[0] if allow_unrecognized else parser.parse_args()


def get_current_pytest():
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")
    return current_test


def pytest_main(file):
    """Runs the pytest file

    :param file: The test file to run
    """
    args = load_command_line_arguments(allow_unrecognized=True)

    # Run the test, -s disables all capturing at -vv increases verbosity
    # -W ignore::DeprecationWarning ignores deprecation warnings that spam the output
    sys.exit(
        pytest.main(
            ["-svv", "-W ignore::DeprecationWarning", "-k", args.test_filter, file]
        )
    )
