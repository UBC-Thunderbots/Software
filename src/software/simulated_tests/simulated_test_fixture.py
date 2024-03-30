import threading
import queue
import argparse
import time
import sys
import os

import pytest
import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from pyqtgraph.Qt import QtCore, QtGui

from software.networking.unix.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.robot_enters_region import RobotEntersRegion

from software.simulated_tests import validation
from software.simulated_tests.tbots_test_runner import TbotsTestRunner
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import MILLISECONDS_PER_SECOND
from software.thunderscope.constants import ProtoUnixIOTypes
from software.thunderscope.binary_context_managers.full_system import FullSystem
from software.thunderscope.binary_context_managers.simulator import Simulator
from software.thunderscope.binary_context_managers.game_controller import Gamecontroller
from software.thunderscope.thunderscope_config import configure_simulated_test_view
from software.thunderscope.replay.proto_logger import ProtoLogger

from software.logger.logger import createLogger

logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
PROCESS_BUFFER_DELAY_S = 0.01
TEST_START_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


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
        :param thunderscope: The thunderscope to use, None if not used
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

    def set_worldState(self, worldstate: WorldState):
        """Sets the simulation worldstate

        Args:
            worldstate (WorldState): proto containing the desired worldstate
        """
        self.simulator_proto_unix_io.send_proto(WorldState, worldstate)

    def time_provider(self):
        """Provide the current time in seconds since the epoch"""

        with self.timestamp_mutex:
            return self.timestamp

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

    def runner(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
        tick_duration_s=0.0166,  # Default to 60hz
        ci_cmd_with_delay=[],
        run_till_end=True,
    ):
        """Run a test

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        :param tick_duration_s: The simulation step duration
        :param ci_cmd_with_delay: A list consisting of a duration, and a
                                tuple forming a ci command
                                {
                                    (time, command, team),
                                    (time, command, team),
                                    ...
                                }
        :param run_till_end: If true, test runs till the end even if eventually validation passes
                             If false, test stops once eventually validation passes and fails if time out
        """

        time_elapsed_s = 0

        eventually_validation_failure_msg = "Test Timed Out"

        while time_elapsed_s < test_timeout_s:
            # get time before we execute the loop
            processing_start_time = time.time()

            # Check for new CI commands at this time step
            for (delay, cmd, team) in ci_cmd_with_delay:
                # If delay matches time
                if delay <= time_elapsed_s:
                    # send command
                    self.gamecontroller.send_ci_input(cmd, team)
                    # remove command from the list
                    ci_cmd_with_delay.remove((delay, cmd, team))

            # Update the timestamp logged by the ProtoLogger
            with self.timestamp_mutex:
                ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                self.timestamp = ssl_wrapper.detection.t_capture

            tick = SimulatorTick(milliseconds=tick_duration_s * MILLISECONDS_PER_SECOND)
            self.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
            time_elapsed_s += tick_duration_s

            while True:
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
                except queue.Empty as empty:
                    # If we timeout, that means full_system missed the last
                    # wrapper and robot status, lets resend it.
                    logger.warning("Fullsystem missed last wrapper, resending ...")

                    ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                    robot_status = self.robot_status_buffer.get(block=False)

                    self.blue_full_system_proto_unix_io.send_proto(
                        SSL_WrapperPacket, ssl_wrapper
                    )
                    self.blue_full_system_proto_unix_io.send_proto(
                        RobotStatus, robot_status
                    )

            # get the time difference after we get the primitive (after any blocking that happened)
            processing_time = time.time() - processing_start_time

            # if the time we have blocked is less than a tick, sleep for the remaining time (for Thunderscope only)
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

            if self.thunderscope:

                # Set the test name
                eventually_validation_proto_set.test_name = self.test_name
                always_validation_proto_set.test_name = self.test_name

                # Send out the validation proto to thunderscope
                self.thunderscope.proto_unix_io_map[ProtoUnixIOTypes.BLUE].send_proto(
                    ValidationProtoSet, eventually_validation_proto_set
                )
                self.thunderscope.proto_unix_io_map[ProtoUnixIOTypes.BLUE].send_proto(
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

    def run_test(
        self,
        always_validation_sequence_set,
        eventually_validation_sequence_set,
        test_timeout_s=3,
        tick_duration_s=0.0166,
        index=0,
        run_till_end=True,
        **kwargs,
    ):
        """
        Helper function to run a test, with thunderscope if enabled
        :param always_validation_sequence_set: validation that should always be true
        :param eventually_validation_sequence_set: validation that should eventually be true
        :param test_timeout_s: how long the test should run before timing out
        :param tick_duration_s: length of a tick
        :param index: index of the current test. default is 0 (invariant test)
                      values can be passed in during aggregate testing for different timeout durations
        :param run_till_end: If true, test runs till the end even if eventually validation passes
                             If false, test stops once eventually validation passes and fails if time out
        """

        test_timeout_duration = (
            test_timeout_s[index] if type(test_timeout_s) == list else test_timeout_s
        )

        # Start the test with a delay to allow the simulator to receive
        # the initial world state. Without this delay, the SimulatorTick
        # message may be received before the initial world state, causing
        # the world to be empty, failing some AlwaysValidations
        # TODO (#2858): Replace delay with an actual feedback from the simulator
        #  for when it has received the initial world state
        time.sleep(TEST_START_DELAY_S)

        # If thunderscope is enabled, run the test in a thread and show
        # thunderscope on this thread. The excepthook is setup to catch
        # any test failures and propagate them to the main thread
        if self.thunderscope:

            run_sim_thread = threading.Thread(
                target=self.runner,
                daemon=True,
                args=[
                    always_validation_sequence_set,
                    eventually_validation_sequence_set,
                    test_timeout_duration,
                    tick_duration_s,
                    [],
                    run_till_end,
                ],
            )
            run_sim_thread.start()
            self.thunderscope.show()
            run_sim_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        # If thunderscope is disabled, just run the test
        else:
            self.runner(
                always_validation_sequence_set,
                eventually_validation_sequence_set,
                test_timeout_duration,
                tick_duration_s,
                run_till_end=run_till_end,
            )


class InvariantTestRunner(SimulatedTestRunner):

    """
    Runs a simulated test only once with a given parameter

    Test passes or fails based on the outcome of this test
    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def run_test(
        self,
        setup=(lambda x: None),
        params=[0],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        **kwargs,
    ):
        """Run an invariant test

        :param setup: Function that sets up the World state and the gamecontroller before running the test
        :param params: List of parameters for each iteration of the test
                        (this method only uses the first element)
        :param inv_always_validation_sequence_set: Validation functions for invariant testing
                                that should hold on every tick
        :param inv_eventually_validation_sequence_set: Validation functions for invariant testing
                                that should eventually be true, before the test ends

        """

        threading.excepthook = self.excepthook

        setup(params[0])

        super().run_test(
            inv_always_validation_sequence_set,
            inv_eventually_validation_sequence_set,
            **kwargs,
        )


class AggregateTestRunner(SimulatedTestRunner):

    """
    Runs a simulated test multiple times with different given parameters

    Result of the test is determined by comparing the number of
    passing iterations to a predetermined acceptable threshold
    """

    def __int__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def run_test(
        self,
        setup=(lambda arg: None),
        params=[],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        **kwargs,
    ):
        """Run an aggregate test

        :param setup: Function that sets up the World state and the gamecontroller before running the test
        :param params: List of parameters for each iteration of the test
        :param ag_always_validation_sequence_set: Validation functions for aggregate testing
                                that should hold on every tick
        :param ag_eventually_validation_sequence_set: Validation functions for aggregate testing
                                that should eventually be true, before the test end
        """

        threading.excepthook = self.excepthook

        failed_tests = 0

        # Runs the test once for each given parameter
        # Catches Assertion Error thrown by failing test and increments counter
        # Calculates overall results and prints them
        for x in range(len(params)):

            setup(params[x])

            try:
                super().run_test(
                    ag_always_validation_sequence_set,
                    ag_eventually_validation_sequence_set,
                    **kwargs,
                )
            except AssertionError:
                failed_tests += 1

        # TODO (#2856) Fix validation and results output

        logger.info(f"{failed_tests} test failed")

        assert failed_tests == 0


def load_command_line_arguments():
    """Load from command line arguments using argpase

    NOTE: Pytest has its own built in argument parser (conftest.py, pytest_addoption)
    but it doesn't seem to play nicely with bazel. We just use argparse instead.

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
    return parser.parse_args()


def pytest_main(file):
    """Runs the pytest file

    :param file: The test file to run

    """
    args = load_command_line_arguments()
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main(["-svv", "-k", args.test_filter, file]))


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
        f"{args.blue_full_system_runtime_dir}/test/{test_name}",
        args.debug_blue_full_system,
        False,
        should_restart_on_crash=False,
    ) as blue_fs, FullSystem(
        f"{args.yellow_full_system_runtime_dir}/test/{test_name}",
        args.debug_yellow_full_system,
        True,
        should_restart_on_crash=False,
    ) as yellow_fs:
        with Gamecontroller(
            supress_logs=(not args.show_gamecontroller_logs)
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
                blue_full_system_proto_unix_io, yellow_full_system_proto_unix_io,
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

            time.sleep(LAUNCH_DELAY_S)

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
                runner = InvariantTestRunner(
                    current_test,
                    tscope,
                    simulator_proto_unix_io,
                    blue_full_system_proto_unix_io,
                    yellow_full_system_proto_unix_io,
                    gamecontroller,
                )

            # Setup proto loggers.
            #
            # NOTE: Its important we use the test runners time provider because
            # test will run as fast as possible with a varying tick rate. The
            # SimulatorTestRunner time provider is tied to the simulators
            # t_capture coming out of the wrapper packet (rather than time.time).
            with ProtoLogger(
                f"{args.blue_full_system_runtime_dir}/logs/{current_test}",
                time_provider=runner.time_provider,
            ) as blue_logger, ProtoLogger(
                f"{args.yellow_full_system_runtime_dir}/logs/{current_test}",
                time_provider=runner.time_provider,
            ) as yellow_logger:

                blue_full_system_proto_unix_io.register_to_observe_everything(
                    blue_logger.buffer
                )
                yellow_full_system_proto_unix_io.register_to_observe_everything(
                    yellow_logger.buffer
                )

                yield runner
                print(
                    f"\n\n To replay this test for the blue team, go to the `src` folder and run \n./tbots.py run thunderscope --blue_log {blue_logger.log_folder}"
                )
                print(
                    f"\n\n To replay this test for the yellow team, go to the `src` folder and run \n./tbots.py run thunderscope --yellow_log {yellow_logger.log_folder}"
                )
