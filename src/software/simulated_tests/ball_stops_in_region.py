import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallStopsInRegion(Validation):
    """Checks if a ball stops in any of the provided regions."""

    def __init__(self, regions=None):
        self.regions = regions if regions else []

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball stops in the provided regions

        :param world: The world msg to validate
        :return: FAILING until a ball stops in any of the regions
                 PASSING when a ball stops in a region
        """
        for region in self.regions:
            if tbots_cpp.contains(
                region, tbots_cpp.createPoint(world.ball.current_state.global_position)
            ) and (
                tbots_cpp.createVector(
                    world.ball.current_state.global_velocity
                ).length()
                <= 0.01
            ):
                return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry containing geometry to visualize
        """
        return create_validation_geometry(self.regions)

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
