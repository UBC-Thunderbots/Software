import argparse
import os
from dataclasses import dataclass

import pytest

from proto.import_all_protos import *
from software.gameplay_tests import fixture
from software.logger.logger import create_logger
from software.thunderscope.test_runner_widget import TestRunnerWidget
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thunderscope_config import (
    configure_field_test_view,
    configure_simulated_test_view,
    initialize_application,
)
from software.thunderscope.thunderscope_types import TScopeWidget, WidgetPosition

logger = create_logger(__name__)

TEST_MODE_RUNTIME_SUBPATH = "test_mode"
"""Fixed runtime subdirectory used for the long-lived test-mode binaries"""


@dataclass
class TestModeSession:
    """The live binaries, ProtoUnixIOs, and Thunderscope used by test mode.

    A single session is created when Thunderscope is launched with --test_mode
    and is reused to bind each selected test to the already-running binaries.
    """

    context: fixture.SessionContext
    thunderscope: Thunderscope
    run_field_test: bool
    is_yellow_friendly: bool
    ci_mode: bool


class _ResultCollector:
    """A pytest plugin that records the pass/fail outcome of a run's tests."""

    def __init__(self) -> None:
        self.outcomes = []

    def pytest_runtest_logreport(self, report) -> None:
        """Record the outcome of each test's call phase (and any setup failures).

        :param report: the pytest TestReport for a phase of a test
        """
        if report.when == "call" or (
            report.when == "setup" and report.outcome != "passed"
        ):
            self.outcomes.append(report.outcome)

    def summary(self, exit_code) -> str:
        """Summarize the recorded outcomes into a status string.

        :param exit_code: the pytest.ExitCode returned by pytest.main
        :return: a human-readable status string
        """
        if not self.outcomes:
            return f"No tests ran (exit code {int(exit_code)})"

        passed = sum(1 for outcome in self.outcomes if outcome == "passed")
        total = len(self.outcomes)
        status = "PASSED" if passed == total and int(exit_code) == 0 else "FAILED"
        return f"{status}: {passed}/{total} passed"


def _reset_session_state(session: TestModeSession) -> None:
    """Halts the AI and clears tactic overrides between runs.

    Keeps each run hermetic and, for field tests, stops the robots once a test
    finishes.

    :param session: the active TestModeSession.
    """
    context = session.context
    try:
        context.gamecontroller.send_gc_command(
            gc_command=Command.Type.HALT, team=Team.UNKNOWN
        )
        empty_tactics = AssignedTacticPlayControlParams()
        context.blue_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, empty_tactics
        )
        context.yellow_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, empty_tactics
        )
    except Exception as exception:
        logger.warning(f"Failed to reset state after test: {exception}")


def run_selected_test(session: TestModeSession, test_path: str) -> str:
    """Runs a single gameplay test bound to the given session.

    Sets fixture.ACTIVE_SESSION so the gameplay_test_runner fixture binds the
    test to the open Thunderscope, runs the test in-process with pytest, then
    clears the session and resets state.

    :param session: the active TestModeSession.
    :param test_path: absolute path to the test file to run.
    :return: a human-readable status string describing the outcome.
    """
    fixture.ACTIVE_SESSION = session
    collector = _ResultCollector()
    try:
        # Pass the fixture module as a plugin so gameplay_test_runner is available
        # regardless of where pytest resolves conftest.py for the source-tree test.
        exit_code = pytest.main(
            [
                test_path,
                "-svv",
                "-p",
                "no:cacheprovider",
                "-W",
                "ignore::DeprecationWarning",
            ],
            plugins=[collector, fixture],
        )
    finally:
        fixture.ACTIVE_SESSION = None
        _reset_session_state(session)
    return collector.summary(exit_code)


def discover_tests(run_field_test: bool) -> list[tuple[str, str]]:
    """Discovers runnable gameplay tests from the workspace source tree.

    Scans for test files that use the gameplay_test_runner fixture, returning
    field tests (files ending in _field_test.py) in field mode and simulated
    tests (other _test.py files) otherwise.

    :param run_field_test: whether to list field tests instead of simulated tests.
    :return: list of (display_name, absolute_path) tuples, sorted by name.
    """
    workspace = os.environ.get("BUILD_WORKSPACE_DIRECTORY")
    if not workspace:
        logger.warning("BUILD_WORKSPACE_DIRECTORY is not set; cannot discover tests")
        return []

    software_dir = os.path.join(workspace, "software")
    tests = []
    for root, _, files in os.walk(software_dir):
        for name in files:
            if not name.endswith("_test.py"):
                continue
            if name.endswith("_field_test.py") != run_field_test:
                continue

            path = os.path.join(root, name)
            try:
                with open(path) as test_file:
                    if "gameplay_test_runner" not in test_file.read():
                        continue
            except OSError:
                continue

            tests.append((os.path.relpath(path, software_dir), path))

    return sorted(tests)


def _build_test_args(main_args) -> argparse.Namespace:
    """Builds a gameplay-test args namespace from thunderscope_main's arguments.

    Maps the subset of test-mode-relevant flags onto the namespace expected by
    the session context managers and runners.

    :param main_args: parsed thunderscope_main command-line arguments.
    :return: a Namespace compatible with the session context managers.
    """
    return argparse.Namespace(
        run_field_test=main_args.run_field_test,
        simulator_runtime_dir=main_args.simulator_runtime_dir,
        blue_full_system_runtime_dir=main_args.blue_full_system_runtime_dir,
        yellow_full_system_runtime_dir=main_args.yellow_full_system_runtime_dir,
        debug_simulator=main_args.debug_simulator,
        debug_blue_full_system=main_args.debug_blue_full_system,
        debug_yellow_full_system=main_args.debug_yellow_full_system,
        enable_realism=main_args.enable_realism,
        enable_thunderscope=True,
        ci_mode=main_args.ci_mode,
        show_gamecontroller_logs=main_args.verbose,
        run_yellow=main_args.run_yellow,
        channel=main_args.channel,
        interface=main_args.interface,
        keyboard_estop=main_args.keyboard_estop,
        disable_communication=main_args.disable_communication,
        layout=main_args.layout,
    )


def launch_test_mode(main_args) -> None:
    """Launches Thunderscope in test mode and runs the Qt event loop.

    Starts the gameplay-test binaries once, builds a Thunderscope view with the
    Test Runner widget, and keeps everything alive so that tests selected in the
    widget run against the already-open Thunderscope.

    :param main_args: parsed thunderscope_main command-line arguments.
    """
    args = _build_test_args(main_args)
    tests = discover_tests(args.run_field_test)

    session_cm = (
        fixture.field_session(args, TEST_MODE_RUNTIME_SUBPATH)
        if args.run_field_test
        else fixture.simulated_session(args, TEST_MODE_RUNTIME_SUBPATH)
    )

    with session_cm as context:
        # Must construct the QApplication before any QWidget (the Test Runner
        # widget) is created. configure_*_view calls this again, but it is
        # idempotent.
        initialize_application()

        # Holds the session so the run callback can reach it; the session is only
        # created after the Thunderscope (which needs the widget) exists.
        session_holder = {}

        def run_callback(test_path: str) -> str:
            return run_selected_test(session_holder["session"], test_path)

        test_runner_widget = TScopeWidget(
            name="Test Runner",
            widget=TestRunnerWidget(tests, run_callback),
            anchor="Logs",
            position=WidgetPosition.ABOVE,
            has_refresh_func=False,
        )

        if args.run_field_test:
            thunderscope = Thunderscope(
                configure_field_test_view(
                    simulator_proto_unix_io=context.simulator_proto_unix_io,
                    blue_full_system_proto_unix_io=context.blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io=context.yellow_full_system_proto_unix_io,
                    yellow_is_friendly=args.run_yellow,
                    extra_widgets=[test_runner_widget],
                ),
                layout_path=main_args.layout,
            )
            fixture.wire_field_keyboard_estop(
                thunderscope, context.robot_communication, context.estop_mode
            )
        else:
            thunderscope = Thunderscope(
                configure_simulated_test_view(
                    blue_full_system_proto_unix_io=context.blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io=context.yellow_full_system_proto_unix_io,
                    simulator_proto_unix_io=context.simulator_proto_unix_io,
                    extra_widgets=[test_runner_widget],
                ),
                layout_path=main_args.layout,
            )

        session_holder["session"] = TestModeSession(
            context=context,
            thunderscope=thunderscope,
            run_field_test=args.run_field_test,
            is_yellow_friendly=args.run_yellow,
            ci_mode=args.ci_mode,
        )

        thunderscope.show()
