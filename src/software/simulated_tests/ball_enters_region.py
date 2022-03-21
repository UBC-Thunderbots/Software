import pytest

import software.geom.geometry as geom
from proto.geometry_pb2 import Angle, AngularVelocity, Point, Vector
from proto.tbots_software_msgs_pb2 import Vision
from proto.validation_pb2 import ValidationGeometry, ValidationProto, ValidationStatus
from proto.world_pb2 import SimulatorTick, World, WorldState

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallEntersRegion(Validation):

    """Checks if a ball enters any of the provided regions."""

    def __init__(self, regions=None):
        self.regions = regions if regions else []

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball enters the provided regions

        :param world: The world msg to validate
        :returns: FAILING until a ball enters any of the regions
                  PASSING when a ball enters
        """
        for region in self.regions:
            if geom.contains(
                region, geom.createPoint(world.ball.current_state.global_position)
            ):
                return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create v alidation geometry from
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry(self.regions)

    def __repr__(self):
        return "Checking ball in regions " + ",".join(
            repr(region) for region in self.regions
        )


(
    BallEventuallyEntersRegion,
    BallEventuallyExitsRegion,
    BallStaysInRegion,
    BallNeverEntersRegion,
) = create_validation_types(BallEntersRegion)
