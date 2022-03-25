import software.geom.geometry as geom
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
        """Checks if all robots halt

        :param world: The world msg to validate
        :returns: FAILING if ball doesn't move forward
                  PASSING if ball moves forward
        """
        validation_status = ValidationStatus.FAILING
        current_ball_position = geom.createPoint(
            world.ball.current_state.global_position
        )

        if current_ball_position.x() > self.last_ball_position.x():
            validation_status = ValidationStatus.PASSING
        self.last_ball_position = current_ball_position
        return validation_status

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        # TODO: visualize
        return create_validation_geometry([])

    def __repr__(self):
        return "Check that all robots halt"


(
    BallEventuallyMovesForward,
    BallEventuallyStopsMovingForward,
    BallAlwaysMovesForward,
    BallNeverMovesBackward,
) = create_validation_types(BallMovesForward)
