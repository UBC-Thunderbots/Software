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
    ):
        """Constructs the validation object

        :param kick_direction: The expected direction the ball should be kicked (tbots_cpp.Angle)
        """
        self.kick_direction = kick_direction

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball has been kicked in the expected direction

        :param world: The world msg to validate
        :return: FAILING if the ball has not been kicked in the expected direction
                 PASSING if the ball has been kicked in the expected direction
        """
        # Convert ball proto to tbots_cpp ball
        ball_pos = tbots_cpp.createPoint(world.ball.current_state.global_position)
        ball_vel = tbots_cpp.createVector(world.ball.current_state.global_velocity)

        ball = tbots_cpp.Ball(ball_pos, ball_vel, tbots_cpp.Timestamp())

        return (
            ValidationStatus.PASSING
            if ball.hasBallBeenKicked(self.kick_direction)
            else ValidationStatus.FAILING
        )

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns arrow geometry showing the expected kick direction

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry containing an arrow in the kick direction
        """
        ball_pos = world.ball.current_state.global_position
        start = tbots_cpp.Point(ball_pos.x_meters, ball_pos.y_meters)
        direction = tbots_cpp.Vector.createFromAngle(self.kick_direction)
        end = start + direction * self.LINE_LENGTH
        return create_validation_geometry([tbots_cpp.Segment(start, end)])

    @override
    def __repr__(self):
        return f"Check that the ball is kicked in direction {self.kick_direction}"


(
    BallEventuallyKickedInDirection,
    _BallEventuallyNotKickedInDirection,  # Doesn't make sense
    _BallAlwaysKickedInDirection,  # Doesn't make sense
    BallNeverKickedInDirection,
) = create_validation_types(BallKickedInDirection)
