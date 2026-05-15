import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class BallKickedInDirection(Validation):
    """Checks if the ball has been kicked in a specific direction"""

    LINE_LENGTH = 0.5

    def __init__(
        self,
        kick_direction,
        min_kick_speed=0.5,
        max_angle_difference_degrees=20,
    ):
        """Constructs the validation object

        :param kick_direction: The expected direction the ball should be kicked (tbots_cpp.Angle)
        :param min_kick_speed: Minimum speed for a valid kick in m/s
        :param max_angle_difference_degrees: Maximum angle difference in degrees
        """
        self.kick_direction = kick_direction
        self.min_kick_speed = min_kick_speed
        self.max_angle_difference = tbots_cpp.Angle.fromDegrees(
            max_angle_difference_degrees
        )

    @override
    def get_validation_status(self, world, simulator_state=None) -> ValidationStatus:
        """Checks if the ball has been kicked in the expected direction

        :param world: The world msg to validate
        :return: FAILING if the ball has not been kicked in the expected direction
                 PASSING if the ball has been kicked in the expected direction
        """
        ball_pos = world.ball.current_state.global_position
        ball_vel = world.ball.current_state.global_velocity

        point = tbots_cpp.Point(ball_pos.x_meters, ball_pos.y_meters)
        vector = tbots_cpp.Vector(
            ball_vel.x_component_meters, ball_vel.y_component_meters
        )

        ball = tbots_cpp.Ball(point, vector, tbots_cpp.Timestamp())

        if ball.hasBallBeenKicked(
            self.kick_direction, self.min_kick_speed, self.max_angle_difference
        ):
            return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns arrow geometry showing the expected kick direction

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry containing an arrow in the kick direction
        """
        ball_pos = world.ball.current_state.global_position
        start = tbots_cpp.Point(ball_pos.x_meters, ball_pos.y_meters)
        direction = tbots_cpp.Vector(1, 0).rotate(self.kick_direction)
        end = start + direction * self.LINE_LENGTH
        return create_validation_geometry([tbots_cpp.Segment(start, end)])

    @override
    def __repr__(self):
        return f"Check that the ball was kicked at {self.kick_direction} (min_speed={self.min_kick_speed}, max_angle_diff={self.max_angle_difference})"


(
    BallEventuallyKickedInDirection,
    BallEventuallyNotKickedInDirection,
    BallAlwaysKickedInDirection,
    BallNeverKickedInDirection,
) = create_validation_types(BallKickedInDirection)
