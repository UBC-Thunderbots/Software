import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallMovesForward(Validation):

    """Checks if ball is moving forward, i.e. in the +x direction"""

    def __init__(self, initial_ball_position):
        self.last_ball_position = initial_ball_position

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if ball is moving forward, i.e. in the +x direction

        :param world: The world msg to validate
        :returns: FAILING if ball doesn't move forward
                  PASSING if ball moves forward
        """
        validation_status = ValidationStatus.FAILING
        current_ball_position = tbots.createPoint(
            world.ball.current_state.global_position
        )

        if current_ball_position.x() > self.last_ball_position.x():
            validation_status = ValidationStatus.PASSING
        self.last_ball_position = current_ball_position
        return validation_status

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) Shows the last ball position line
        """
        return create_validation_geometry(
            [
                tbots.Rectangle(
                    tbots.Point(
                        self.last_ball_position.x(),
                        tbots.Field(world.field).fieldBoundary().yMin(),
                    ),
                    tbots.Point(
                        self.last_ball_position.x() + 0.01,
                        tbots.Field(world.field).fieldBoundary().yMax(),
                    ),
                )
            ]
        )

    def __repr__(self):
        return "Check that the ball moves forward"


(
    BallEventuallyMovesForward,
    BallEventuallyStopsMovingForward,
    BallAlwaysMovesForward,
    BallNeverMovesForward,
) = create_validation_types(BallMovesForward)
