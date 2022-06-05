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
        print("creating validation gemoetry")

        segments = []
        for robot in world.friendly_team.team_robots:
            robot_x = robot.current_state.global_position.x_meters
            robot_y = robot.current_state.global_position.y_meters
            validation_line_centre_x = (start_x + ball.current_state.global_velocity.x_component_meters)*MILLIMETERS_PER_METER
            validation_line_centre_y = (start_y + ball.current_state.global_velocity.y_component_meters)*MILLIMETERS_PER_METER
            robot_radians = robot.current_state.global_orientation.radians
            start_angle = robot_radians - math.pi / 2
            start_x = validation_line_centre_x + ROBOT_MAX_RADIUS_MILLIMETERS * math.cos(start_angle) * MILLIMETERS_PER_METER
            start_y = validation_line_centre_y + ROBOT_MAX_RADIUS_MILLIMETERS * math.sin(start_angle) * MILLIMETERS_PER_METER

            end_angle = robot_radians + math.pi / 2
            end_x = validation_line_centre_x + ROBOT_MAX_RADIUS_MILLIMETERS * math.cos(end_angle) * MILLIMETERS_PER_METER
            end_y = validation_line_centre_y + ROBOT_MAX_RADIUS_MILLIMETERS * math.sin(end_angle) * MILLIMETERS_PER_METER
            segments.Add([createPoint(start_x, start_y), createPoint(end_x, end_y)])

        return create_validation_geometry(
            [
                tbots.Segment(points[0], points[1])
                for points in segments
            ]
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
