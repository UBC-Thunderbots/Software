import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallMoves(Validation):
    """Checks if ball is moving forward or backward, i.e. in the +x direction"""

    def __init__(self, initial_ball_position, direction):
        """
        Constructs the validation
        :param initial_ball_position: the initial position of the ball
        :param direction: True if checking for forward, false if backward
        """
        self.last_ball_position = initial_ball_position
        self.direction = direction

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if ball is moving forward or backward

        :param world: The world msg to validate
        :returns: FAILING if ball doesn't move in the direction
                  PASSING if ball moves in the direction
        """
        validation_status = ValidationStatus.FAILING
        current_ball_position = tbots.createPoint(
            world.ball.current_state.global_position
        )

        if (self.direction and current_ball_position.x() >= self.last_ball_position.x()) or (not self.direction and current_ball_position.x() <= self.last_ball_position.x()):
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
        return "Check that the ball moves " + "forward" if self.direction else "backward"


class BallMovesForward(BallMoves):

    """Checks if ball is moving forward, i.e. in the +x direction"""

    def __init__(self, initial_ball_position):
        super().__init__(initial_ball_position, True)


(
    BallEventuallyMovesForward,
    BallEventuallyStopsMovingForward,
    BallAlwaysMovesForward,
    BallNeverMovesForward,
) = create_validation_types(BallMovesForward)


class BallMovesInDirectionInRegions(BallMoves):
    """Checks if ball is moving in a direction in certain regions"""

    def __init__(self, initial_ball_position, direction, regions=[]):
        super().__init__(initial_ball_position, direction)
        self.regions = regions

    def get_validation_status(self, world) -> ValidationStatus:

        for region in self.regions:
            if tbots.contains(
                    region, tbots.createPoint(world.ball.current_state.global_position)
            ):
                return super().get_validation_status(world)

        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) Shows the last ball position line, and the regions the ball should be moving in
        """
        return create_validation_geometry(
            self.regions
        )

    def __repr__(self):
        return "Check that the ball moves " + "forward" if self.direction else "backward" + " in regions "  + ",".join(
            repr(region) for region in self.regions
        )

(
    BallEventuallyMovesInDirectionInRegions,
    BallEventuallyStopsMovingInDirectionInRegions,
    BallAlwaysMovesInDirectionInRegions,
    BallNeverMovesInDirectionInRegions,
) = create_validation_types(BallMovesInDirectionInRegions)


