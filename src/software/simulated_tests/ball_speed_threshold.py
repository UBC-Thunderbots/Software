import software.python_bindings as tbots
from proto.import_all_protos import *
from software.py_constants import *
from software.simulated_tests.speed_threshold_helpers import *

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
        if get_ball_speed(world.ball) == 0:
            return create_validation_geometry([])

        ball_x, ball_y = get_current_ball_position(world.ball)
        ball_angle = get_current_ball_angle(world.ball)

        validation_centre_x, validation_centre_y = get_validation_centre_position(
            ball_x, ball_y, self.speed_threshold, ball_angle
        )

        (
            validation_start_x,
            validation_end_x,
            validation_start_y,
            validation_end_y,
        ) = get_validation_line_endpoints(
            validation_centre_x, validation_centre_y, ball_angle
        )

        return create_validation_geometry(
            [
                tbots.Segment(
                    tbots.Point(validation_start_x, validation_start_y),
                    tbots.Point(validation_end_x, validation_end_y),
                )
            ]
        )

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
