import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotHalt(Validation):

    """Checks if the friendly robots' are halted."""

    def __init__(self):
        """
        :param speed_threshold: The speed threshold
        """

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the friendly robots' are stopped

        :param world: The world msg to validate
        :returns: FAILING if the friendly robots' speed is above 1e-3
                  PASSING if the friendly robots' speed is at or below 1e-3
        """
        for robot in world.friendly_team.team_robots:
            if (
                    tbots.createVector(robot.current_state.global_velocity).length()
                    > 1e-3
            ):
                return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        # TODO (#2556): visualize
        return create_validation_geometry([])

    def __repr__(self):
        return "Check that the friendly robots' halted "


(
    RobotEventuallyHalt,
    RobotEventuallyMoves,
    RobotAlwaysHalt,
    RobotAlwaysMoves,
) = create_validation_types(RobotHalt)
