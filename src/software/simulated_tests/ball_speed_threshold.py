import software.python_bindings as tbots
from proto.import_all_protos import *
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
        self.MILLIMETERS_PER_METER = 1000
        self.ROBOT_MAX_RADIUS_MILLIMETERS = 90

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
        ball_x = world.ball.current_state.global_position.x_meters * self.MILLIMETERS_PER_METER
        ball_y = world.ball.current_state.global_position.y_meters * self.MILLIMETERS_PER_METER
        
        ball_xdir = world.ball.current_state.global_velocity.x_component_meters * self.MILLIMETERS_PER_METER
        ball_ydir = world.ball.current_state.global_velocity.y_component_meters * self.MILLIMETERS_PER_METER
        
        ball_angle = math.pi / 2
        if ball_xdir is not 0:
            ball_angle = math.atan2(ball_ydir, ball_xdir)
                    
        validation_line_centre_x = (ball_x + self.MILLIMETERS_PER_METER * self.speed_threshold * math.cos(ball_angle))
        validation_line_centre_y = (ball_y + self.MILLIMETERS_PER_METER * self.speed_threshold * math.sin(ball_angle))
        
        # default: if ball_angle is between 0 and PI
        start_x = validation_line_centre_x - math.sin(ball_angle) * self.MILLIMETERS_PER_METER / 2
        start_y = validation_line_centre_y + math.cos(ball_angle) * self.MILLIMETERS_PER_METER / 2
        end_x = validation_line_centre_x + math.sin(ball_angle) * self.MILLIMETERS_PER_METER / 2
        end_y = validation_line_centre_y - math.cos(ball_angle) * self.MILLIMETERS_PER_METER / 2

        if ball_angle > math.pi or ball_angle < 0:
            start_y = validation_line_centre_y - math.cos(ball_angle) * self.MILLIMETERS_PER_METER / 2
            end_y = validation_line_centre_y + math.cos(ball_angle) * self.MILLIMETERS_PER_METER / 2

        return create_validation_geometry(
            [        
                tbots.Segment(tbots.Point(start_x, start_y), tbots.Point(end_x, end_y))
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
