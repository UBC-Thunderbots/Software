import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallMovesFromRest(Validation):
    """Checks if ball has moved threshold meters from initial position"""

    def __init__(self, position, threshold=0.05):
        """
        :param position: initial position of the ball
        :param threshold: distance for which ball is considered to have moved
        """
        self.initial_ball_position = position
        self.threshold = threshold

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if ball has moved threshold meters from initial position. Default is 0.05m.

        :param world: The world msg to validate
        :returns: FAILING if ball doesn't move according to RoboCup rules
                  PASSING if ball moves according to RoboCup rules
        """
        validation_status = ValidationStatus.FAILING
        current_ball_position = tbots_cpp.createPoint(
            world.ball.current_state.global_position
        )

        if (
            self.initial_ball_position - current_ball_position
        ).length() > self.threshold:
            validation_status = ValidationStatus.PASSING

        return validation_status

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) Shows the last ball position line
        """
        return create_validation_geometry(
            [tbots_cpp.Circle(self.initial_ball_position, 0.05)]
        )

    def __repr__(self):
        return "Check that the ball moves from rest"


(
    BallEventuallyMovesFromRest,
    BallStopsMovingFromRest,
    BallAlwaysMovesFromRest,
    BallNeverMovesFromRest,
) = create_validation_types(BallMovesFromRest)
