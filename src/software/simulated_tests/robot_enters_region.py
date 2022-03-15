import pytest

import software.geom.geometry as tbots_geom
from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from proto.tbots_software_msgs_pb2 import Vision
from proto.validation_pb2 import ValidationGeometry, ValidationProto, ValidationStatus
from proto.vision_pb2 import BallState, RobotState
from proto.world_pb2 import SimulatorTick, World, WorldState

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotEntersRegion(Validation):

    """Checks if a Robot enters any of the provided regions."""

    def __init__(self, regions=[]):
        self.regions = regions

    def get_validation_status(self, vision) -> ValidationStatus:
        """Checks if _any_ robot enters the provided regions

        :param vision: The vision msg to validate
        :returns: FAILING until a robot enters any of the regions
                  PASSING when a robot enters
        """
        for region in self.regions:
            for robot_id, robot_states in vision.robot_states.items():
                if tbots_geom.contains(
                    region, tbots_geom.createPoint(robot_states.global_position)
                ):
                    return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, vision) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param vision: The vision msg to validate
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry(self.regions)

    def __repr__(self):
        return "Checking regions " + ",".join(repr(region) for region in self.regions)


(
    RobotEventuallyEntersRegion,
    RobotEventuallyExitsRegion,
    RobotStaysInRegion,
    RobotNeverEntersRegion,
) = create_validation_types(RobotEntersRegion)
