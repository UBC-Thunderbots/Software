import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotSpeedThreshold(Validation):

    """Checks if the friendly robots' speed is at or above some threshold."""

    def __init__(self, speed_threshold):
        """
        :param speed_threshold: The speed threshold
        """
        self.speed_threshold = speed_threshold

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the friendly robots' speed is at or above some threshold

        :param world: The world msg to validate
        :returns: FAILING if the friendly robots' speed is below some threshold
                  PASSING if the friendly robots' speed is at or above some threshold
        """
        for robot in world.friendly_team.team_robots:
            if (
                tbots.createVector(robot.current_state.global_velocity).length()
                < self.speed_threshold
            ):
                return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        # TODO (#2556): visualize
        return create_validation_geometry([])

    def __repr__(self):
        return "Check that the friendly robots' speed is at or above above " + str(
            self.speed_threshold
        )


(
    RobotSpeedEventuallyAtOrAboveThreshold,
    RobotSpeedEventuallyBelowThreshold,
    RobotSpeedAlwaysAtOrAboveThreshold,
    RobotSpeedAlwaysBelowThreshold,
) = create_validation_types(RobotSpeedThreshold)
