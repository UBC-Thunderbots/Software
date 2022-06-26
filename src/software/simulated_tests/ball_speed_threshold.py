import software.python_bindings as tbots
from proto.import_all_protos import *
from software.py_constants import *
import math

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)

class BallSpeedThreshold(Validation):

    """Checks if the ball speed is at or above some threshold."""

    def __init__(self, speed_threshold):
        """
        :param speed_threshold: The speed threshold in m/s
        """
        self.speed_threshold = speed_threshold
        self.VALIDATION_LINE_SCALE_FACTOR = 300

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball speed is at or above some threshold

        :param world: The world msg to validate
        :returns: FAILING if the ball speed is below some threshold
                  PASSING if the ball speed is at or above some threshold
        """
        if (
            tbots.createVector(world.ball.current_state.global_velocity).length()
            >= self.speed_threshold
        ):
            return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        ball_speed = math.sqrt(
            world.ball.current_state.global_velocity.x_component_meters ** 2
            + world.ball.current_state.global_velocity.y_component_meters ** 2
        )
        if ball_speed == 0:
            return create_validation_geometry([])

        ball_x = (
            world.ball.current_state.global_position.x_meters
            * MILLIMETERS_PER_METER
        )
        ball_y = (
            world.ball.current_state.global_position.y_meters
            * MILLIMETERS_PER_METER
        )

        ball_angle = math.pi / 2
        if world.ball.current_state.global_velocity.x_component_meters is not 0:
            ball_angle = math.atan2(
                world.ball.current_state.global_velocity.y_component_meters,
                world.ball.current_state.global_velocity.x_component_meters,
            )

        validation_centre_x = (
            ball_x
            + MILLIMETERS_PER_METER * self.speed_threshold * math.cos(ball_angle)
        )
        validation_centre_y = (
            ball_y
            + MILLIMETERS_PER_METER * self.speed_threshold * math.sin(ball_angle)
        )

        endpoints = self.get_validation_line_endpoints(
            validation_centre_x, validation_centre_y, ball_angle
        )
        start_x = endpoints[0]
        end_x = endpoints[1]
        start_y = endpoints[2]
        end_y = endpoints[3]

        return create_validation_geometry(
            [tbots.Segment(tbots.Point(start_x, start_y), tbots.Point(end_x, end_y))]
        )
    def get_validation_line_endpoints(self, validation_centre_x, validation_centre_y, ball_angle):
        start_x = (
            validation_centre_x
            - math.sin(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )
        end_x = (
            validation_centre_x
            + math.sin(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )

        start_y = (
            validation_centre_y
            + math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )
        end_y = (
            validation_centre_y
            - math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
        )

        if ball_angle > math.pi or ball_angle < 0:
            start_y = (
                validation_centre_y
                + math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
            )
            end_y = (
                validation_centre_y
                - math.cos(ball_angle) * self.VALIDATION_LINE_SCALE_FACTOR
            )

        return [start_x, end_x, start_y, end_y]

    def __repr__(self):
        return "Check that the ball speed is at or above above " + str(
            self.speed_threshold
        )

(
    BallSpeedEventuallyAtOrAboveThreshold,
    BallSpeedEventuallyBelowThreshold,
    BallSpeedAlwaysAtOrAboveThreshold,
    BallSpeedAlwaysBelowThreshold,
) = create_validation_types(BallSpeedThreshold)
