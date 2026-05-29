import threading
import queue
import argparse
import time
import sys
import os

import pytest
from proto.import_all_protos import *

from software.simulated_tests.validation import validation
from software.simulated_tests.tbots_test_runner import TbotsTestRunner
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import MILLISECONDS_PER_SECOND
from software.thunderscope.binary_context_managers.full_system import FullSystem
from software.thunderscope.binary_context_managers.simulator import Simulator
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from software.thunderscope.thunderscope_config import configure_simulated_test_view
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer

from software.logger.logger import create_logger
from typing import override

logger = create_logger(__name__)

LAUNCH_DELAY_S = 2
WORLD_BUFFER_TIMEOUT = 10
PROCESS_BUFFER_DELAY_S = 0.1
PAUSE_AFTER_FAIL_DELAY_S = 5
SECONDS_PER_MILLISECOND = 0.001


class SimulatedTestRunner(TbotsTestRunner):
    """Run a simulated test"""

    def __init__(
        self,
        test_name,
        thunderscope,
        simulator_proto_unix_io,
        blue_full_system_proto_unix_io,
        yellow_full_system_proto_unix_io,
        gamecontroller,
    ):
        """Initialize the SimulatorTestRunner

        :param test_name: The name of the test to run
        :param thunderscope: The Thunderscope to use, None if not used
        :param simulator_proto_unix_io: The simulator proto unix io to use
        :param blue_full_system_proto_unix_io: The blue full system proto unix io to use
        :param yellow_full_system_proto_unix_io: The yellow full system proto unix io to use
        :param gamecontroller: The gamecontroller context managed instance
        """
        super(SimulatedTestRunner, self).__init__(
            test_name,
            thunderscope,
            blue_full_system_proto_unix_io,
            yellow_full_system_proto_unix_io,
            gamecontroller,
        )
        self.simulator_proto_unix_io = simulator_proto_unix_io

    @override
    def set_world_state(self, worldstate: WorldState):
        """Sets the simulation worldstate

        :param worldstate: proto containing the desired worldstate
        """
        self.simulator_proto_unix_io.send_proto(WorldState, worldstate)

    def excepthook(self, args):
        """This function is _critical_ for show_thunderscope to work.
        If the test Thread will raises an exception we won't be able to close
        the window from the main thread.

        :param args: The args passed in from the hook
        """
        self.__stopper(delay=PAUSE_AFTER_FAIL_DELAY_S)
        self.last_exception = args.exc_value
        raise self.last_exception

    def __stopper(self, delay=PROCESS_BUFFER_DELAY_S):
        """Stop running the test

        :param delay: How long to wait before closing everything, defaults
                      to PROCESS_BUFFER_DELAY_S to minimize buffer warnings
        """
        time.sleep(delay)

        if self.thunderscope:
            self.thunderscope.close()

    def sync_setup(self, setup, param):
        """Run setup until simulator has received game state

        :param setup: Function that sets up the world state
        :param param: Parameter passed into setup
        """
        world_state_received_buffer = ThreadSafeBuffer(1, WorldStateReceivedTrigger)
        self.simulator_proto_unix_io.register_observer(
            WorldStateReceivedTrigger, world_state_received_buffer
        )

        retries = 0
        MAX_RETRIES = 100  # 10 seconds total at 0.1s sleep
        while retries < MAX_RETRIES:
            setup(param)

            try:
                world_state_received_buffer.get(
                    block=True, timeout=0.1, return_cached=False
                )
                return
            except queue.Empty:
                retries += 1

        raise TimeoutError("Timed out waiting for Simulator to receive world state")

    def runner(
        self,
        always_validation_sequence_set,
        eventually_validation_sequence_set,
        test_timeout_s=3,
        tick_duration_s=1.0 / 60.0,
        ci_cmd_with_delay=[],
        run_till_end=False,
        **kwargs,
    ):
        """Ticks the simulation forward while running the validations

        :param eventually_validation_sequence_set: validation set that must eventually be true
        :param always_validation_sequence_set: validation set that must always be true
        :param test_timeout_s: how long the test will run
        :param tick_duration_s: how long each simulation step will be
        :param run_till_end: if the test should run till the test timeout even if a pass condition is reached
        :param ci_cmd_with_delay: A list consisting of tuples with a duration and CI command, e.g.
                                  [(1.0, Command.Type.NORMAL_START, Team.BLUE)]
        """
        time_elapsed_s = 0
        eventually_validation_failure_msg = "Test Timed Out"
        eventually_validation_proto_set = None

        while time_elapsed_s < test_timeout_s:
            start_time = time.time()

            # Check for new CI commands at this time step
            for delay, cmd, team in ci_cmd_with_delay[:]:
                # If delay matches time
                if delay <= time_elapsed_s:
                    # send command
                    self.gamecontroller.send_gc_command(gc_command=cmd, team=team)
                    # remove command from the list
                    ci_cmd_with_delay.remove((delay, cmd, team))

            tick = SimulatorTick(milliseconds=tick_duration_s * MILLISECONDS_PER_SECOND)
            self.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
            time_elapsed_s += tick_duration_s

            retry_count = 0
            MAX_RETRIES = 5
            while retry_count < MAX_RETRIES:
                try:
                    world = self.world_buffer.get(
                        block=True, timeout=WORLD_BUFFER_TIMEOUT, return_cached=False
                    )

                    # We block until the timeout for the new primitives from AI. if not found still,
                    # the SSL Wrapper packet is resent in a loop until we actually get a primitive set from AI
                    # Otherwise, if the AI misses the first SSL Wrapper packet and doesn't start
                    # the simulated test will continue to tick forward, causes syncing issues with the AI
                    self.primitive_set_buffer.get(
                        block=True, timeout=WORLD_BUFFER_TIMEOUT, return_cached=False
                    )

                    break
                except queue.Empty:
                    retry_count += 1
                    logger.warning(
                        f"Timeout waiting for world/primitives (retry {retry_count}/{MAX_RETRIES}). Resending SSL Wrapper."
                    )
                    # No world or primitives was found within the given timeout. Re-send the SSL wrapper packet and try again.
                    for packet in self.ssl_wrapper_buffer.get_all():
                        self.blue_full_system_proto_unix_io.send_proto(
                            SSL_WrapperPacket, packet
                        )
                        self.yellow_full_system_proto_unix_io.send_proto(
                            SSL_WrapperPacket, packet
                        )

            if retry_count == MAX_RETRIES:
                raise TimeoutError("Timed out waiting for world/primitive updates from AI/Simulator")

            processing_time = time.time() - start_time
            if self.thunderscope and tick_duration_s > processing_time:
                time.sleep(tick_duration_s - processing_time)

            # Validate
            (
                eventually_validation_proto_set,
                always_validation_proto_set,
            ) = validation.run_validation_sequence_sets(
                world,
                eventually_validation_sequence_set,
                always_validation_sequence_set,
            )

            # Set the test name
            eventually_validation_proto_set.test_name = self.test_name
            always_validation_proto_set.test_name = self.test_name

            # Send out the validation proto to the full system
            # for visualization and logging for replays.
            if self.is_yellow_friendly:
                self.yellow_full_system_proto_unix_io.send_proto(
                    ValidationProtoSet, eventually_validation_proto_set
                )
                self.yellow_full_system_proto_unix_io.send_proto(
                    ValidationProtoSet, always_validation_proto_set
                )
            else:
                self.blue_full_system_proto_unix_io.send_proto(
                    ValidationProtoSet, eventually_validation_proto_set
                )
                self.blue_full_system_proto_unix_io.send_proto(
                    ValidationProtoSet, always_validation_proto_set
                )

            # Check that all always validations are always valid
            validation.check_validation(always_validation_proto_set)

            if not run_till_end:
                try:
                    # Check that all eventually validations are eventually valid
                    validation.check_validation(eventually_validation_proto_set)
                    self.__stopper()
                    return
                except AssertionError as e:
                    eventually_validation_failure_msg = str(e)

        if not run_till_end:
            raise AssertionError(eventually_validation_failure_msg)

        # Check that all eventually validations are eventually valid
        validation.check_validation(eventually_validation_proto_set)
        self.__stopper()

    @override
    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        setup=None,
        test_timeout_s=3,
        tick_duration_s=1.0 / 60.0,
        ci_cmd_with_delay=[],
        run_till_end=False,
        **kwargs,
    ):
        """Begins validating a test based on incoming world protos

        :param eventually_validation_sequence_set: validation set that must eventually be true
        :param always_validation_sequence_set: validation set that must always be true
        :param setup: initialization function for this test
        :param test_timeout_s: how long the test will run
        """
        # Set the hook for exception handling so that we can close the thunderscope
        # instance should one exist
        sys.excepthook = self.excepthook

        # Only run setup if provided and if we are not being called from AggregateTestRunner
        # (which handles its own setup loop)
        if setup and "params" not in kwargs:
            self.sync_setup(setup, self)

        self.runner(
            always_validation_sequence_set=always_validation_sequence_set,
            eventually_validation_sequence_set=eventually_validation_sequence_set,
            test_timeout_s=test_timeout_s,
            tick_duration_s=tick_duration_s,
            ci_cmd_with_delay=ci_cmd_with_delay,
            run_till_end=run_till_end,
            **kwargs,
        )


class AggregateTestRunner(SimulatedTestRunner):
    """A test runner for aggregate tests.
    These tests are a collection of invariant tests. If any of the invariant tests fail,
    the aggregate test fails.
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    @override
    def run_test(
        self,
        setup=(lambda arg: None),
        params=[],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        **kwargs,
    ):
        """Begins validating a test based on incoming world protos. Runs the
        invariant test first, then the aggregate test.

        :param setup: initialization function for this test
        :param params: List of parameters for each iteration of the test
        :param ag_eventually_validation_sequence_set: validation set for aggregate test that must eventually be true
        :param ag_always_validation_sequence_set: validation set for aggregate test that must always be true
        """
        sys.excepthook = self.excepthook

        failed_tests = 0

        # Create a copy of kwargs without 'params' to avoid double-setup in SimulatedTestRunner
        clean_kwargs = {k: v for k, v in kwargs.items() if k != "params"}

        for x in range(len(params)):
            super().sync_setup(setup, params[x])

            try:
                super().run_test(
                    always_validation_sequence_set=ag_always_validation_sequence_set,
                    eventually_validation_sequence_set=ag_eventually_validation_sequence_set,
                    **clean_kwargs,
                )
            except AssertionError:
                failed_tests += 1

        # TODO (#2856) Fix validation and results output

        logger.info(f"{failed_tests} test failed")

        assert failed_tests == 0


def load_command_line_arguments(allow_unrecognized: bool = False):
    """Load in command-line arguments using argparse

    NOTE: Pytest has its own built in argument parser (conftest.py, pytest_addoption)
    but it doesn't seem to play nicely with bazel. We just use argparse instead.

    :param allow_unrecognized: if true, does not raise an error for unrecognized arguments
    """
    parser = argparse.ArgumentParser(description="Run simulated pytests")
    parser.add_argument(
        "--enable_thunderscope", action="store_true", help="enable thunderscope"
    )
    parser.add_argument(
        "--aggregate", action="store_true", default=False, help="Run aggregate test"
    )
    parser.add_argument(
        "--simulator_runtime_dir",
        type=str,
        help="simulator runtime directory",
        default="/tmp/tbots",
    )
    parser.add_argument(
        "--blue_full_system_runtime_dir",
        type=str,
        help="blue full_system runtime directory",
        default="/tmp/tbots/blue",
    )
    parser.add_argument(
        "--yellow_full_system_runtime_dir",
        type=str,
        help="yellow full_system runtime directory",
        default="/tmp/tbots/yellow",
    )
    parser.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )
    parser.add_argument(
        "--debug_blue_full_system",
        action="store_true",
        default=False,
        help="Debug blue full_system",
    )
    parser.add_argument(
        "--debug_yellow_full_system",
        action="store_true",
        default=False,
        help="Debug yellow full_system",
    )
    parser.add_argument(
        "--debug_simulator",
        action="store_true",
        default=False,
        help="Debug the simulator",
    )
    parser.add_argument(
        "--visualization_buffer_size",
        action="store",
        type=int,
        default=5,
        help="How many packets to buffer while rendering",
    )
    parser.add_argument(
        "--show_gamecontroller_logs",
        action="store_true",
        default=False,
        help="Show gamecontroller logs",
    )
    parser.add_argument(
        "--test_filter",
        action="store",
        default="",
        help="The test filter, if not specified all tests will run. "
        + "See https://docs.pytest.org/en/latest/how-to/usage.html#specifying-tests-selecting-tests",
    )
    parser.add_argument(
        "--enable_realism",
        action="store_true",
        default=False,
        help="Use realism in the simulator",
    )
    parser.add_argument(
        "--ci_mode",
        action="store_true",
        default=False,
        help="Run in CI mode (faster execution)",
    )
    return parser.parse_known_args()[0] if allow_unrecognized else parser.parse_args()


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


@pytest.fixture
def simulated_test_runner():
    args = load_command_line_arguments()
    tscope = None

    aggregate = args.aggregate

    simulator_proto_unix_io = ProtoUnixIO()
    yellow_full_system_proto_unix_io = ProtoUnixIO()
    blue_full_system_proto_unix_io = ProtoUnixIO()

    # Grab the current test name to store the proto log for the test case
    current_test = os.environ.get("PYTEST_CURRENT_TEST").split(":")[-1].split(" ")[0]
    current_test = current_test.replace("]", "")
    current_test = current_test.replace("[", "-")

    test_name = current_test.split("-")[0]

    # Launch all binaries
    with Simulator(
        f"{args.simulator_runtime_dir}/test/{test_name}",
        args.debug_simulator,
        args.enable_realism,
    ) as simulator, FullSystem(
        "software/unix_full_system",
        f"{args.blue_full_system_runtime_dir}/test/{test_name}",
        args.debug_blue_full_system,
        False,
        should_restart_on_crash=False,
        running_in_realtime=not args.ci_mode,
    ) as blue_fs, FullSystem(
        "software/unix_full_system",
        f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
        args.debug_yellow_full_system,
        True,
        should_restart_on_crash=False,
        running_in_realtime=not args.ci_mode,
    ) as yellow_fs:
        with Gamecontroller(
            suppress_logs=(not args.show_gamecontroller_logs)
        ) as gamecontroller:
            blue_fs.setup_proto_unix_io(blue_full_system_proto_unix_io)
            yellow_fs.setup_proto_unix_io(yellow_full_system_proto_unix_io)
            simulator.setup_proto_unix_io(
                simulator_proto_unix_io,
                blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io,
                ProtoUnixIO(),
            )
            gamecontroller.setup_proto_unix_io(
                blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                simulator_proto_unix_io=simulator_proto_unix_io,
            )

            # If we want to run thunderscope, inject the proto unix ios
            # and start the test
            if args.enable_thunderscope:
                tscope = Thunderscope(
                    configure_simulated_test_view(
                        blue_full_system_proto_unix_io=blue_full_system_proto_unix_io,
                        yellow_full_system_proto_unix_io=yellow_full_system_proto_unix_io,
                        simulator_proto_unix_io=simulator_proto_unix_io,
                    ),
                    layout_path=args.layout,
                )

            # Even in CI mode, give a small delay for processes to start up
            actual_launch_delay = 0.5 if args.ci_mode else LAUNCH_DELAY_S
            time.sleep(actual_launch_delay)

            runner = None

            # Initialise the right runner based on which testing mode is selected
            if aggregate:
                runner = AggregateTestRunner(
                    current_test,
                    tscope,
                    simulator_proto_unix_io,
                    blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io,
                    gamecontroller,
                )
            else:
                runner = SimulatedTestRunner(
                    current_test,
                    tscope,
                    simulator_proto_unix_io,
                    blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io,
                    gamecontroller,
                )

            yield runner
