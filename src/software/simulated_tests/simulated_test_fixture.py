import logging
import threading
import time

import pytest

import software.geom.geometry as tbots_geom
from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.robot_status_msg_pb2 import RobotStatus
from proto.sensor_msg_pb2 import SensorProto
from proto.tactic_pb2 import (AssignedTacticPlayControlParams, GoalieTactic,
                              Tactic)
from proto.tbots_software_msgs_pb2 import Vision
from proto.validation_pb2 import (ValidationGeometry, ValidationProto,
                                  ValidationStatus)
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import SimulatorTick, World, WorldState
from pyqtgraph.Qt import QtCore, QtGui

from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.eventually_validation.robot_enters_region import \
    RobotEntersRegion
from software.simulated_tests.full_system import FullSystem
from software.simulated_tests.er_force_simulator import ErForceSimulator
from software.thunderscope.thunderscope import Thunderscope

from software.logger.logger import createLogger
logger = createLogger(__name__)


def run_validation_sequence_sets(
    vision, eventually_validation_sequence_set, always_validation_sequence_set
):
    """Given both eventually and always validation sequence sets, (and vision)
    run validation and aggregate the results in a validation proto.

    :raises AssertionError: If the test fails
    :param vision: Vision to validate with
    :param eventually_validation_sequence_set:
            A collection of sequences of eventually validations to validate.
    :param always_validation_sequence_set:
            A collection of sequences of always validations to validate.

    :returns: ValidationProto, error_msg

    """

    # Proto that stores validation geometry and validation status
    validation_proto = ValidationProto()
    error_msg = None

    # Validate
    for validation_sequence in eventually_validation_sequence_set:

        # We only want to check the first
        for validation in validation_sequence:
            status = validation.get_validation_status(vision)

            validation_proto.status.append(status)
            validation_proto.geometry.append(validation.get_validation_geometry(vision))

            # If the current validation is pending, we don't care about
            # the next one. Keep evaluating until this one passes.
            if status == ValidationStatus.FAILING:
                break

            # If the validation has passed, continue
            # this line is not needed, but just added to be explicit
            if status == ValidationStatus.PASSING:
                continue

    return validation_proto, error_msg


class TacticTestRunner(object):

    """Run a tactic"""

    def __init__(self, launch_delay_s=0.1, base_unix_path="/tmp/tbots"):
        """Initialize the TacticTestRunner

        :param launch_delay_s: How long to wait after launching the processes

        """
        self.thunderscope = Thunderscope()
        self.simulator = ErForceSimulator()
        self.yellow_full_system = FullSystem()
        time.sleep(0.1)

        self.validation_sender = ThreadedUnixSender(base_unix_path + "/validation")

    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
        tick_duration_s=0.0166, # Default to 60hz
        open_thunderscope=True,
    ):
        """Run a test

        :param always_validation_sequence_set: Validation functions that should
                                hold on every tick
        :param eventually_validation_sequence_set: Validation that should
                                eventually be true, before the test ends
        :param test_timeout_s: The timeout for the test, if any eventually_validations
                                remain after the timeout, the test fails.
        :param tick_duration_s: The simulation step duration
        :param open_thunderscope: If true, thunderscope opens and the test runs
                                  in realtime
        """

        def __stopper():
            self.simulator.simulator_process.kill()
            self.yellow_full_system.full_system_process.kill()
            self.simulator.simulator_process.wait()
            self.yellow_full_system.full_system_process.wait()
            self.thunderscope.close()

        def __runner():
            time_elapsed_s = 0
            cached_vision = Vision()

            while time_elapsed_s < test_timeout_s:

                ssl_wrapper = self.simulator.get_ssl_wrapper_packet()
                self.simulator.tick(tick_duration_s * 1000)
                time_elapsed_s += tick_duration_s

                if open_thunderscope:
                    time.sleep(tick_duration_s)

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
                validation, error_msg = run_validation_sequence_sets(
                    vision,
                    eventually_validation_sequence_set,
                    always_validation_sequence_set,
                )

                # NOTE: The following line will raise AssertionError(
                # on validation failure that will propagate to the main
                # thread through the excepthook
                if error_msg:
                    raise AssertionError(error_msg)

                # Send out the validation proto to thunderscope
                self.validation_sender.send(validation)

                # Step the primtives
                self.simulator.send_yellow_primitive_set_and_vision(
                    vision, self.yellow_full_system.get_primitive_set(),
                )

            __stopper()

        def excepthook(args):
            """This function is _critical_ for pytest to work. Threads in python
            manage their own exceptions, but we want the assertion error
            (and any other exceptions) to propagate to the main test so that
            pytest can fail correctly.

            We re-raise the exception on the main-thread with a log to indicate
            where the exception occurred

            :param args: The args passed in from the hook

            """
            logger.critical("Exception triggered in {}".format(args.thread))
            raise args.exc_value

        threading.excepthook = excepthook

        run_sim_thread = threading.Thread(target=__runner)
        run_sim_thread.start()

        if open_thunderscope:
            self.thunderscope.show()


@pytest.fixture
def tactic_runner():
    runner = TacticTestRunner()
    yield runner
