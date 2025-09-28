import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class BallEntersRegion(Validation):
    """Checks if a ball enters any of the provided regions."""

    def __init__(self, regions=None):
        self.regions = regions if regions else []
        self.ball_position = None

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball enters the provided regions

        :param world: The world msg to validate
        :return: FAILING until a ball enters any of the regions
                 PASSING when a ball enters
        """
        self.ball_position = world.ball.current_state.global_position
        for region in self.regions:
            if tbots_cpp.contains(
                region, tbots_cpp.createPoint(world.ball.current_state.global_position)
            ):
                return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create v alidation geometry from
        :return: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry(self.regions)

    @override
    def __repr__(self):
        return (
            "Checking ball in regions "
            + ",".join(repr(region) for region in self.regions)
            + ", ball position: "
            + str(self.ball_position)
            if self.ball_position
            else ""
        )


(
    BallEventuallyEntersRegion,
    BallEventuallyExitsRegion,
    BallAlwaysStaysInRegion,
    BallNeverEntersRegion,
) = create_validation_types(BallEntersRegion)
