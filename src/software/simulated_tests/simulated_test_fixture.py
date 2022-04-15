import threading
import argparse
import time
import queue

import pytest
import software.python_bindings as tbots
from proto.import_all_protos import *

from pyqtgraph.Qt import QtCore, QtGui

from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.robot_enters_region import RobotEntersRegion

from software.simulated_tests import validation
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import MILLISECONDS_PER_SECOND

from software.logger.logger import createLogger

logger = createLogger(__name__)

PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


class SimulatorTestRunner(object):

    """Run a simulated test"""

    def __init__(self, thunderscope, show_thunderscope=True):
        """Initialize the SimulatorTestRunner

        :param thunderscope: The thunderscope to use
        :param show_thunderscope: If true, thunderscope opens and the test runs in realtime

        """

        self.thunderscope = thunderscope
        self.show_thunderscope = show_thunderscope
        self.world_buffer = queue.Queue()
        self.last_exception = None

        # Only validate on the blue worlds
        self.thunderscope.blue_full_system_proto_unix_io.register_observer(
            World, self.world_buffer
        )

    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
        tick_duration_s=0.0166,  # Default to 60hz
    ):
        """Run a test

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        :param tick_duration_s: The simulation step duration

        """

        def __stopper(delay=PROCESS_BUFFER_DELAY_S):
            """Stop running the test

            :param delay: How long to wait before closing everything, defaults
                          to PROCESS_BUFFER_DELAY_S to minimize buffer warnings

            """
            time.sleep(delay)
            self.thunderscope.close()

        def __runner():
            """Step simulation, full_system and run validation
            """

            time_elapsed_s = 0

            while time_elapsed_s < test_timeout_s:

                tick = SimulatorTick(
                    milliseconds=tick_duration_s * MILLISECONDS_PER_SECOND
                )
                self.thunderscope.simulator_proto_unix_io.send_proto(
                    SimulatorTick, tick
                )
                time_elapsed_s += tick_duration_s

                if self.show_thunderscope:
                    time.sleep(tick_duration_s)

                world = self.world_buffer.get()

                # Validate
                (
                    eventually_validation_proto_set,
                    always_validation_proto_set,
                ) = validation.run_validation_sequence_sets(
                    world,
                    eventually_validation_sequence_set,
                    always_validation_sequence_set,
                )

                if self.show_thunderscope:
                    # Send out the validation proto to thunderscope
                    self.thunderscope.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, eventually_validation_proto_set
                    )
                    self.thunderscope.blue_full_system_proto_unix_io.send_proto(
                        ValidationProtoSet, always_validation_proto_set
                    )

                # Check that all always validations are always valid
                validation.check_validation(always_validation_proto_set)

            # Check that all eventually validations are eventually valid
            validation.check_validation(eventually_validation_proto_set)

            __stopper()

        def excepthook(args):
            """This function is _critical_ for show_thunderscope to work.
            If the test Thread will raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook

            """

            __stopper(delay=PAUSE_AFTER_FAIL_DELAY_S)
            self.last_exception = args.exc_value
            raise self.last_exception

        threading.excepthook = excepthook

        # If thunderscope is enabled, run the test in a thread and show
        # thunderscope on this thread. The excepthook is setup to catch
        # any test failures and propagate them to the main thread
        if self.show_thunderscope:

            run_sim_thread = threading.Thread(target=__runner, daemon=True)
            run_sim_thread.start()
            self.thunderscope.show()
            run_sim_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        # If thunderscope is disabled, just run the test
        else:
            __runner()


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
        "--simulator_runtime_dir",
        type=str,
        help="simulator runtime directory",
        default="/tmp/tbots",
    )
    parser.add_argument(
        "--blue_fullsystem_runtime_dir",
        type=str,
        help="blue fullsystem runtime directory",
        default="/tmp/tbots/blue",
    )
    parser.add_argument(
        "--yellow_fullsystem_runtime_dir",
        type=str,
        help="yellow fullsystem runtime directory",
        default="/tmp/tbots/yellow",
    )
    parser.add_argument(
        "--layout",
        action="store",
        help="Which layout to run, if not specified the last layout will run",
    )
    parser.add_argument(
        "--debug_fullsystem",
        action="store_true",
        default=False,
        help="Debug fullsystem",
    )
    parser.add_argument(
        "--debug_simulator",
        action="store_true",
        default=False,
        help="Debug the simulator",
    )
    return parser.parse_args()


@pytest.fixture
def simulated_test_runner():
    args = load_command_line_arguments()

    thunderscope = Thunderscope(
        args.simulator_runtime_dir,
        args.blue_fullsystem_runtiem_dir,
        args.yellow_fullsystem_runtime_dir,
        debug_fullsystem=args.debug_fullsystem,
        debug_simulator=args.debug_simulator,
    )
    thunderscope.load_saved_layout(args.layout)

    runner = SimulatorTestRunner(
        thunderscope=thunderscope, show_thunderscope=args.enable_thunderscope,
    )

    with thunderscope:
        yield runner
