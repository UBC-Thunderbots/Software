import logging

import pytest

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - [%(levelname)s] - [%(threadName)s] - %(name)s - (%(filename)s).%(funcName)s(%(lineno)d) - %(message)s",
)
logger = logging.getLogger(__name__)
from typing import List, Set

import software.geom.geometry as tbots_geom
from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.robot_status_msg_pb2 import RobotStatus
from proto.sensor_msg_pb2 import SensorProto
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from proto.tbots_software_msgs_pb2 import Vision
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import (
    SimulatorTick,
    ValidationGeometry,
    ValidationProto,
    ValidationStatus,
    World,
    WorldState,
)
from pyqtgraph.Qt import QtCore, QtGui

from software.networking.threaded_unix_sender import ThreadedUnixSender
from software.simulated_tests.full_system_wrapper import FullSystemWrapper
from software.simulated_tests.standalone_simulator_wrapper import (
    StandaloneSimulatorWrapper,
)
from software.thunderscope.thunderscope import Thunderscope


class Validation(object):

    """A validation function"""

    def __init__(self, flipped = False):
        self.flipped = flipped

    def get_validation_status(self, vision) -> ValidationStatus:
        return self.__flip(self._get_private_validation_status(vision)) if self.flipped else self._get_private_validation_status(vision)

    def get_validation_geometry(self, vision) -> ValidationGeometry:
        raise NotImplementedError("get_validation_geometry is not implemented")

    def get_failure_message(self):
        raise NotImplementedError("get_failure_message is not implemented")

    def _get_private_validation_status(self, vision) -> ValidationStatus:
        raise NotImplementedError("get_validation_status is not implemented")

    def _flip(self, validation_status) -> ValidationStatus:
        if validation_status == ValidationStatus.PASSING:
            return ValidationStatus.FAILING
        if validation_status == ValidationStatus.FAILING:
            return ValidationStatus.PASSING


class EventuallyValidation(Validation):
    pass


class AlwaysValidation(Validation):
    pass


# TODO finalize naming
ValidationSequence = List[Validation]
ValidationSequenceSet = Set[ValidationSequence]


def create_validation_geometry(geometry=[]) -> ValidationGeometry:
    """Given a list of (vectors, polygons, circles), creates a ValidationGeometry
    proto containing the protobuf representations

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
