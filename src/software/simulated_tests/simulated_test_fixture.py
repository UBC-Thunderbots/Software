import sys
import threading
import time

import pytest
import logging

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - [%(threadName)s] - %(name)s - (%(filename)s).%(funcName)s(%(lineno)d) - %(message)s",
)
logger = logging.getLogger(__name__)
from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from software.networking.threaded_unix_sender import ThreadedUnixSender
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.robot_status_msg_pb2 import RobotStatus
from proto.sensor_msg_pb2 import SensorProto
from proto.tbots_software_msgs_pb2 import Vision
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import (
    SimulatorTick,
    World,
    WorldState,
    ValidationProto,
    ValidationStatus,
    ValidationGeometry,
)
from pyqtgraph.Qt import QtCore, QtGui

import software.simulated_tests.python_bindings as py
import software.geom.geometry as tbots_geom
from typing import Set, List

from software.simulated_tests.full_system_wrapper import FullSystemWrapper
from software.simulated_tests.standalone_simulator_wrapper import (
    StandaloneSimulatorWrapper,
)
from software.thunderscope.thunderscope import Thunderscope

# TODO move to validation files
class Validation(object):

    """A validation function that should eventually be true"""

    def validate(self, vision) -> ValidationStatus:
        raise NotImplementedError("validate is not implemented")

    def get_validation_geometry(self, vision) -> ValidationGeometry:
        raise NotImplementedError("get_validation_geometry is not implemented")

    def get_failure_message(self):
        raise NotImplementedError("get_failure_message is not implemented")


class EventuallyValidation(Validation):
    pass


class AlwaysValidation(Validation):
    pass


# TODO finalize naming
ValidationSequence = List[Validation]
ValidationSequenceSet = Set[ValidationSequence]


class RobotEntersRegion(EventuallyValidation):

    """Checks if a Robot enters any of the provided regions."""

    def __init__(self, regions=[]):
        self.regions = regions

    def validate(self, vision) -> ValidationStatus:
        """Checks if _any_ robot enters the provided regions

        :param vision: The vision msg to validate
        :returns: PENDING until a robot enters any of the regions
                  PASS when a robot enters
        """
        for region in self.regions:
            for robot_id, robot_states in vision.robot_states.items():
                if tbots_geom.contains(
                    region, tbots_geom.createPoint(robot_states.global_position)
                ):
                    return ValidationStatus.PASS

        return ValidationStatus.PENDING

    def get_validation_geometry(self, vision) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param vision: The vision msg to validate
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry(self.regions)

    def get_failure_message(self):
        return "Robot didn't enter any of these regions: {}".format(self.regions)


def run_validation_sequence_sets(
    vision, eventually_validation_sequence_set, always_validation_sequence_set
):
    """Given both eventually and always validation sequence sets, (and vision)
    run validation and aggregate the results in a validation proto.

    :raises AssertionError: If the test fails
    :param vision: Vision to validate with
    :param eventually_validation_sequence_set:
            A collection of sequences of EventuallyValidation to validate.
    :param always_validation_sequence_set:
            A collection of sequences of AlwaysValidation to validate.

    """

    # Proto that stores validation geometry and validation status
    validation_proto = ValidationProto()

    # Validate
    for validation_sequence in eventually_validation_sequence_set:

        # We only want to check the first
        for validation in validation_sequence:
            status = validation.validate(vision)

            validation_proto.status.append(status)
            validation_proto.geometry.append(validation.get_validation_geometry(vision))

            # If the validation function failed, raise an
            # AssertionError so that
            if status == ValidationStatus.FAIL:
                raise AssertionError(validation.get_failure_message())

            # If the current validation is pending, we don't care about
            # the next one. Keep evaluating until this one passes.
            if status == ValidationStatus.PENDING:
                break

            # If the validation has passed, continue
            # this line is not needed, but just added to be explicit
            if status == ValidationStatus.PASS:
                continue

    return validation_proto


class TacticTestRunner(object):

    """Run a tactic"""

    def __init__(self, launch_delay_s=0.1, base_unix_path="/tmp/tbots"):
        """Initialize the TacticTestRunner

        :param launch_delay_s: How long to wait after launching 

        """
        self.thunderscope = Thunderscope()
        self.simulator = StandaloneSimulatorWrapper()
        self.yellow_full_system = FullSystemWrapper()

        self.validation_sender = ThreadedUnixSender(base_unix_path + "/validation")

        time.sleep(launch_delay_s)

    def run_test(
        self,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=3,
        tick_duration_s=0.01,
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
            self.simulator.standalone_simulator_process.kill()
            self.yellow_full_system.full_system_process.kill()
            self.simulator.standalone_simulator_process.wait()
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

                # NOTE: The following line will raise AssertionError(
                # on validation failure that will propagate to the main
                # thread through the excepthook
                validation = run_validation_sequence_sets(
                    vision,
                    eventually_validation_sequence_set,
                    always_validation_sequence_set,
                )

                # Send out the validation proto to thunderscope
                self.validation_sender.send(validation)
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


def create_validation_geometry(geometry=[]) -> ValidationGeometry:
    """Given a list of (vectors, polygons, circles), creates
    a ValidationGeometry proto

    :param geometry: A list of geom
    :returns: ValidationGeometry

    """

    validation_geometry = ValidationGeometry()

    CREATE_PROTO_DISPATCH = {
        tbots_geom.Vector.__name__: tbots_geom.createVectorProto,
        tbots_geom.Polygon.__name__: tbots_geom.createPolygonProto,
        tbots_geom.Rectangle.__name__: tbots_geom.createPolygonProto,
        tbots_geom.Circle.__name__: tbots_geom.createCircleProto,
    }

    ADD_TO_VALIDATION_GEOMETRY_DISPATCH = {
        tbots_geom.Vector.__name__: validation_geometry.vectors.append,
        tbots_geom.Polygon.__name__: validation_geometry.polygons.append,
        tbots_geom.Rectangle.__name__: validation_geometry.polygons.append,
        tbots_geom.Circle.__name__: validation_geometry.circles.append,
    }

    for geom in geometry:
        ADD_TO_VALIDATION_GEOMETRY_DISPATCH[type(geom).__name__](
            CREATE_PROTO_DISPATCH[type(geom).__name__](geom)
        )

    return validation_geometry


@pytest.fixture
def tactic_runner():
    runner = TacticTestRunner()
    yield runner


@pytest.mark.parametrize(
    "goalie_starting_position,ball_starting_position",
    [
        ((4.2, 0), (-2, 1)),
        ((4.2, 0.4), (-2, -1)),
        ((4.2, -0.4), (-2, 0.1)),
        ((-4.2, 0.4), (-2, -1)),
        ((-4.2, -0.4), (-2, 0.1)),
        ((-4.2, 0.2), (-2, -0.1)),
        ((-4.2, -0.2), (-2, 1)),
        ((-4.2, 0.2), (-2, -0.1)),
        ((-4.2, -0.5), (-2, 2)),
        ((-4.2, -0.5), (-2, -2)),
    ],
)
def test_goalie_blocks_shot(
    goalie_starting_position, ball_starting_position, tactic_runner
):
    # Setup Robot
    tactic_runner.simulator.setup_yellow_robots([goalie_starting_position])

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].goalie.CopyFrom(
        GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )
    tactic_runner.yellow_full_system.send_tactic_override(params)

    # Setup ball with initial velocity using our software/geom
    tactic_runner.simulator.setup_ball(
        ball_position=tbots_geom.Point(*ball_starting_position),
        ball_velocity=(
            tbots_geom.Field().friendlyGoalCenter()
            - tbots_geom.Vector(ball_starting_position[0], ball_starting_position[1])
        )
        .toVector()
        .normalize(5),  # TODO we should try a range of speeds
    )

    # Validation
    eventually_sequences = [
        [
            # Goalie should be in the defense area
            RobotEntersRegion(regions=[tbots_geom.Field().friendlyDefenseArea()])
        ]
    ]

    tactic_runner.run_test(eventually_validation_sequence_set=eventually_sequences)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-svv"]))
