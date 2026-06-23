import argparse
import threading
from dataclasses import dataclass, field

import pytest

from proto.import_all_protos import *
from software.gameplay_tests import fixture
from software.gameplay_tests.util import discover_tests
from software.logger.logger import create_logger
from software.thunderscope.test_runner_widget import TestRunnerWidget
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thunderscope_config import initialize_application
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
    cancel_event: threading.Event = field(default_factory=threading.Event)


class ResultCollector:
    """A pytest plugin that records the pass/fail outcome of a run's tests."""

    def __init__(self, cancel_event) -> None:
        self.cancel_event = cancel_event
        self.outcomes = []

    def pytest_runtest_setup(self, item) -> None:
        """Abort the whole run (skipping remaining tests) once it is cancelled.

        A single run executes every test in the file, so this stops the next
        test from starting after the user has selected a different test to run.

        :param item: the pytest test item about to be set up
        """
        if self.cancel_event.is_set():
            pytest.exit("Test run cancelled", returncode=pytest.ExitCode.INTERRUPTED)

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


def run_selected_test(session: TestModeSession, test_path: str) -> str:
    """Runs a single gameplay test bound to the given session.

    :param session: the active TestModeSession.
    :param test_path: absolute path to the test file to run.
    :return: a human-readable status string describing the outcome.
    """
    fixture.ACTIVE_SESSION = session
    # Clear once per run (not per test in the file) so a cancel set during the
    # run stops every remaining test, not just the current one.
    session.cancel_event.clear()
    collector = ResultCollector(session.cancel_event)
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
    return collector.summary(exit_code)


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

    :param main_args: parsed thunderscope_main command-line arguments.
    """
    args = _build_test_args(main_args)
    tests = discover_tests()

    session_cm = (
        fixture.field_session(args, TEST_MODE_RUNTIME_SUBPATH)
        if args.run_field_test
        else fixture.simulated_session(args, TEST_MODE_RUNTIME_SUBPATH)
    )

    with session_cm as context:
        # Must construct the QApplication before any QWidget
        initialize_application()

        session = None

        def run_callback(test_path: str) -> str:
            return run_selected_test(session, test_path)

        def cancel_callback() -> None:
            session.cancel_event.set()

        test_runner_widget = TScopeWidget(
            name="Test Runner",
            widget=TestRunnerWidget(tests, run_callback, cancel_callback),
            anchor="Logs",
            position=WidgetPosition.ABOVE,
            has_refresh_func=False,
        )

        if args.run_field_test:
            thunderscope = fixture.build_field_test_thunderscope(
                context,
                args.run_yellow,
                main_args.layout,
                extra_widgets=[test_runner_widget],
            )
        else:
            thunderscope = fixture.build_simulated_test_thunderscope(
                context, main_args.layout, extra_widgets=[test_runner_widget]
            )

        session = TestModeSession(
            context=context,
            thunderscope=thunderscope,
            run_field_test=args.run_field_test,
            is_yellow_friendly=args.run_yellow,
            ci_mode=args.ci_mode,
        )

        thunderscope.show()
