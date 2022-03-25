import software.geom.geometry as tbots_geom
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotsHalt(Validation):

    """Checks if all robots halt."""

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if all robots halt

        :param world: The world msg to validate
        :returns: FAILING until all robots halt
                  PASSING when all robots halt
        """
        for robot in world.friendly_team.team_robots:
            if (tbots_geom.createVector(robot.current_state.global_velocity).length()<0.05):
                return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        # TODO: visualize
        return create_validation_geometry([])

    def __repr__(self):
        return "Check that all robots halt"


(
    RobotsEventuallyHalt,
    RobotsEventuallyMove,
    RobotsAlwaysHalted,
    RobotsAlwaysMoving,
) = create_validation_types(RobotsHalt)
