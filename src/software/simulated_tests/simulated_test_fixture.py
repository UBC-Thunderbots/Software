import threading
import queue
import argparse
import time
import sys
import os

import pytest
import software.python_bindings as tbots
from proto.import_all_protos import *

from pyqtgraph.Qt import QtCore, QtGui

from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.robot_enters_region import RobotEntersRegion

from software.simulated_tests import validation
from software.thunderscope.thunderscope import Thunderscope
from software.thunderscope.thread_safe_buffer import ThreadSafeBuffer
from software.thunderscope.proto_unix_io import ProtoUnixIO
from software.py_constants import MILLISECONDS_PER_SECOND
from software.thunderscope.binary_context_managers import (
    FullSystem,
    Simulator,
    Gamecontroller,
)
from software.thunderscope.replay.proto_logger import ProtoLogger

from software.logger.logger import createLogger
from software.simulated_tests.pytest_main import load_command_line_arguments

logger = createLogger(__name__)

LAUNCH_DELAY_S = 0.1
WORLD_BUFFER_TIMEOUT = 0.5
PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


class SimulatorTestRunner(object):
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

        super(SimulatorTestRunner, self).__init__(test_name,
                                                  thunderscope,
                                                  simulator_proto_unix_io,
                                                  blue_full_system_proto_unix_io,
                                                  yellow_full_system_proto_unix_io,
                                                  gamecontroller)

    def set_tactics(self, tactics:AssignedTacticPlayControlParams, team:Team):

        if team == Team.BLUE:
            self.blue_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams, tactics
            )
        else:
            self.yellow_full_system_proto_unix_io.send_proto(
                AssignedTacticPlayControlParams, tactics
            )

    def set_play(self, play:Play, team:Team):
        if team == Team.BLUE:
            self.blue_full_system_proto_unix_io.send_proto(Play, play)

        else:
            self.yellow_full_system_proto_unix_io.send_proto(Play, play)


    def set_worldState(self, worldstate : WorldState):
        self..simulator_proto_unix_io.send_proto(
            WorldState,
            worldstate
        )

    def time_provider(self):
        """Provide the current time in seconds since the epoch"""

        with self.timestamp_mutex:
            return self.timestamp

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

            if self.thunderscope:
                self.thunderscope.close()

        def __runner():
            """Step simulation, full_system and run validation
            """

            time_elapsed_s = 0

            while time_elapsed_s < test_timeout_s:

                # Update the timestamp logged by the ProtoLogger
                with self.timestamp_mutex:
                    ssl_wrapper = self.ssl_wrapper_buffer.get(block=False)
                    self.timestamp = ssl_wrapper.detection.t_capture

                tick = SimulatorTick(
                    milliseconds=tick_duration_s * MILLISECONDS_PER_SECOND
                )
                self.simulator_proto_unix_io.send_proto(SimulatorTick, tick)
                time_elapsed_s += tick_duration_s

                if self.thunderscope:
                    time.sleep(tick_duration_s)

                while True:
                    try:
                        world = self.world_buffer.get(
                            block=True, timeout=WORLD_BUFFER_TIMEOUT
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
        if self.thunderscope:

            run_sim_thread = threading.Thread(target=__runner, daemon=True)
            run_sim_thread.start()
            self.thunderscope.show()
            run_sim_thread.join()

            if self.last_exception:
                pytest.fail(str(self.last_exception))

        # If thunderscope is disabled, just run the test
        else:
            __runner()
