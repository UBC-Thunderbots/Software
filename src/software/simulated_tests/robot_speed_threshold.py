import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from software.py_constants import *
from software.simulated_tests.speed_threshold_helpers import *


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
                tbots_cpp.createVector(robot.current_state.global_velocity).length()
                < self.speed_threshold
            ):
                return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        segments = []
        for robot in world.friendly_team.team_robots:
            robot_x, robot_y = get_current_robot_position(robot)
            robot_angle = robot.current_state.global_orientation.radians

            validation_centre_x, validation_centre_y = get_validation_centre_position(
                robot_x, robot_y, self.speed_threshold, robot_angle
            )

            (
                validation_start_x,
                validation_end_x,
                validation_start_y,
                validation_end_y,
            ) = get_validation_line_endpoints(
                validation_centre_x, validation_centre_y, robot_angle
            )

            segments.append(
                [
                    tbots_cpp.Point(validation_start_x, validation_start_y),
                    tbots_cpp.Point(validation_end_x, validation_end_y),
                ]
            )

        return create_validation_geometry(
            [tbots_cpp.Segment(points[0], points[1]) for points in segments]
        )

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
