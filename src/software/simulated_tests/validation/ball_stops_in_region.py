import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
    get_ball_pos,
    get_ball_vel,
)
from typing import override


class BallStopsInRegion(Validation):
    """Checks if a ball stops in any of the provided regions."""

    def __init__(self, regions=None):
        self.regions = regions if regions else []

    @override
    def get_validation_status(self, world, simulator_state=None) -> ValidationStatus:
        """Checks if the ball stops in the provided regions

        :param world: The world msg to validate
        :return: FAILING until a ball stops in any of the regions
                 PASSING when a ball stops in a region
        """
        for region in self.regions:
            if tbots_cpp.contains(
                region, get_ball_pos(world, simulator_state)
            ) and get_ball_vel(world, simulator_state).length() <= 0.01:
                return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry containing geometry to visualize
        """
        return create_validation_geometry(self.regions)

    @override
    def __repr__(self):
        return "Checking ball stops in regions " + ",".join(
            repr(region) for region in self.regions
        )


(
    BallEventuallyStopsInRegion,
    BallEventuallyMovesInRegion,
    BallAlwaysStopsInRegion,
    BallNeverStopsInRegion,
) = create_validation_types(BallStopsInRegion)
