import threading
import argparse
import time

import pytest
import software.geom.geometry as tbots_geom
from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.robot_status_msg_pb2 import RobotStatus
from proto.sensor_msg_pb2 import SensorProto
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from proto.tbots_software_msgs_pb2 import Vision
from proto.validation_pb2 import ValidationGeometry, ValidationProto, ValidationStatus
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import SimulatorTick, World, WorldState
from software.constants import SECONDS_TO_MS
from pyqtgraph.Qt import QtCore, QtGui

from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.eventually_validation.robot_enters_region import (
    RobotEntersRegion,
)

from software.simulated_tests import validation
from software.simulated_tests.full_system import FullSystem
from software.simulated_tests.er_force_simulator import ErForceSimulator
from software.thunderscope.thunderscope import Thunderscope

from software.logger.logger import createLogger

logger = createLogger(__name__)

PROCESS_BUFFER_DELAY_S = 0.01
PAUSE_AFTER_FAIL_DELAY_S = 3


class TacticTestRunner(object):

    """Run a tactic"""

    def __init__(
        self, launch_delay_s=0.1, enable_thunderscope=True, runtime_dir="/tmp/tbots"
    ):
        """Initialize the TacticTestRunner

        :param launch_delay_s: How long to wait after launching the processes
        :param enable_thunderscope: If true, thunderscope opens and the test runs
                                  in realtime
        :param runtime_dir: Directory to open sockets, store logs and any output files

        """
        self.enable_thunderscope = enable_thunderscope

        if self.enable_thunderscope:
            self.thunderscope = Thunderscope()
            self.validation_sender = ThreadedUnixSender(runtime_dir + "/validation")

        self.simulator = ErForceSimulator()
        self.yellow_full_system = FullSystem()
        time.sleep(launch_delay_s)

        self.last_exception = None

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

            # Close everything
            self.simulator.simulator_process.kill()
            self.yellow_full_system.full_system_process.kill()
            self.simulator.simulator_process.wait()
            self.yellow_full_system.full_system_process.wait()

            if self.enable_thunderscope:
                self.thunderscope.close()

        def __runner():
            """Step simulation, full_system and run validation
            """

            time_elapsed_s = 0
            cached_vision = Vision()

            while time_elapsed_s < test_timeout_s:

                ssl_wrapper = self.simulator.get_ssl_wrapper_packet()
                self.simulator.tick(tick_duration_s * SECONDS_TO_MS)
                time_elapsed_s += tick_duration_s

                if self.enable_thunderscope:
                    time.sleep(tick_duration_s)
                else:
                    # TODO (#2518) we should remove this sleep and use
                    # blocking calls to make this loop run as fast as it
                    # possibly can.
                    time.sleep(PROCESS_BUFFER_DELAY_S)

                # Send the sensor_proto and get vision
                self.yellow_full_system.send_sensor_proto(
                    self.simulator.get_yellow_sensor_proto(ssl_wrapper)
                )
                vision = self.yellow_full_system.get_vision()

                # IRL, the primitive executor running on real robots will
                # "step" on the last frame they received. We try to emulate the
                # same behaviour by caching the vision if we don't receive one
                # from full_system this tick.
                if vision is None:
                    vision = cached_vision
                else:
                    cached_vision = vision

                # Validate
                validation_proto_set = validation.run_validation_sequence_sets(
                    vision,
                    eventually_validation_sequence_set,
                    always_validation_sequence_set,
                )

                if self.enable_thunderscope:
                    # Send out the validation proto to thunderscope
                    self.validation_sender.send(validation_proto_set)

                # Check that all always validations are always valid
                validation.check_always_validation(validation_proto_set)

                # Step the primtives
                self.simulator.send_yellow_primitive_set_and_vision(
                    vision, self.yellow_full_system.get_primitive_set(),
                )

            # Check that all eventually validations are eventually valid
            validation.check_eventually_validation(validation_proto_set)

            __stopper()

        def excepthook(args):
            """This function is _critical_ for enable_thunderscope to work.
            If the test Thread will raises an exception we won't be able to close
            the window from the main thread.

            :param args: The args passed in from the hook

            """

            __stopper(delay=PAUSE_AFTER_FAIL_DELAY_S)
            self.last_exception = args.exc_value

        threading.excepthook = excepthook

        # If thunderscope is enabled, run the test in a thread and show
        # thunderscope on this thread. The excepthook is setup to catch
        # any test failures and propagate them to the main thread
        if self.enable_thunderscope:

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
    but it doesn't seem to play nicely with bazel. We juse use argparse instead.

    """
    parser = argparse.ArgumentParser(description="Process some integers.")
    parser.add_argument(
        "--enable_thunderscope", action="store_true", help="enable the visualizer"
    )
    return parser.parse_args()


@pytest.fixture
def tactic_runner():
    args = load_command_line_arguments()
    runner = TacticTestRunner(enable_thunderscope=args.enable_thunderscope)
    yield runner
