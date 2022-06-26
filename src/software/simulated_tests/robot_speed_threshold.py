import software.python_bindings as tbots
from proto.import_all_protos import *
from software.py_constants import *

import math

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
        self.VALIDATION_LINE_SCALE_FACTOR = 300

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
        segments = []
        for robot in world.friendly_team.team_robots:
            robot_x = (
                robot.current_state.global_position.x_meters
                * MILLIMETERS_PER_METER
            )
            robot_y = (
                robot.current_state.global_position.y_meters
                * MILLIMETERS_PER_METER
            )
            robot_angle = robot.current_state.global_orientation.radians

            validation_centre_x = (
                robot_x
                + MILLIMETERS_PER_METER
                * self.speed_threshold
                * math.cos(robot_angle)
            )
            validation_centre_y = (
                robot_y
                + MILLIMETERS_PER_METER
                * self.speed_threshold
                * math.sin(robot_angle)
            )

            endpoints = self.get_validation_line_endpoints(
                validation_centre_x, validation_centre_y, robot_angle
            )
            start_x = endpoints[0]
            end_x = endpoints[1]
            start_y = endpoints[2]
            end_y = endpoints[3]

            segments.append([tbots.Point(start_x, start_y), tbots.Point(end_x, end_y)])

        return create_validation_geometry(
            [tbots.Segment(points[0], points[1]) for points in segments]
        )

    def get_validation_line_endpoints(
        self, validation_centre_x, validation_centre_y, robot_angle
    ):
        start_x = (
            validation_centre_x
            - math.sin(robot_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )
        end_x = (
            validation_centre_x
            + math.sin(robot_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )

        start_y = (
            validation_centre_y
            + math.cos(robot_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )
        end_y = (
            validation_centre_y
            - math.cos(robot_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )

        if robot_angle > math.pi or robot_angle < 0:
            start_y = (
                validation_centre_y
                + math.cos(robot_angle) * self.VALIDATION_LINE_SCALE_FACTOR
            )
            end_y = (
                validation_centre_y
                - math.cos(robot_angle) * self.VALIDATION_LINE_SCALE_FACTOR
            )

        return [start_x, end_x, start_y, end_y]

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
