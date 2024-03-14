import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallMovesXAxis(Validation):
    """Checks if ball is moving forward or backward in the +x direction"""

    def __init__(self, initial_ball_position: tbots.Point, moving_in_pos_x: bool):
        """
        Constructs the validation
        :param initial_ball_position: the initial position of the ball
        :param moving_in_pos_x: True if checking for +x direction, false if checking for -x direction
        """
        self.last_ball_position = initial_ball_position
        self.moving_in_pos_x = moving_in_pos_x
        self.max_displacement_so_far = None

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if ball is moving forward or backward

        :param world: The world msg to validate
        :returns: FAILING if ball doesn't move in the direction
                  PASSING if ball moves in the direction
        """
        current_ball_position = tbots.createPoint(
            world.ball.current_state.global_position
        ).x()

        # if max displacement is not set or current ball is moving in the right direction
        # set it and return PASSING
        if self.max_displacement_so_far is None or (
            (
                self.moving_in_pos_x
                and current_ball_position > self.max_displacement_so_far
            )
            or (
                not self.moving_in_pos_x
                and current_ball_position < self.max_displacement_so_far
            )
        ):
            self.max_displacement_so_far = current_ball_position
            return ValidationStatus.PASSING

        # if max displacement is set and current ball is in the wrong direction too far
        # beyond a threshold, return FAILING
        if (
            self.moving_in_pos_x
            and current_ball_position < self.max_displacement_so_far - 0.1
        ) or (
            not self.moving_in_pos_x
            and current_ball_position > self.max_displacement_so_far + 0.1
        ):
            return ValidationStatus.FAILING

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
        return (
            "Check that the ball moves " + "forward"
            if self.moving_in_pos_x
            else "backward"
        )


class BallMovesForward(BallMovesXAxis):

    """Checks if ball is moving forward, i.e. in the +x direction"""

    def __init__(self, initial_ball_position):
        super().__init__(initial_ball_position, True)


(
    BallEventuallyMovesForward,
    BallEventuallyStopsMovingForward,
    BallAlwaysMovesForward,
    BallNeverMovesForward,
) = create_validation_types(BallMovesForward)


class BallMovesInDirectionInRegions(BallMovesXAxis):
    """Checks if ball is moving in a direction in certain regions"""

    def __init__(self, initial_ball_position, moving_in_pos_x, regions=[]):
        super().__init__(initial_ball_position, moving_in_pos_x)
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
        return create_validation_geometry(self.regions)

    def __repr__(self):
        return (
            "Check that the ball moves " + "forward"
            if self.moving_in_pos_x
            else "backward"
            + " in regions "
            + ",".join(repr(region) for region in self.regions)
        )


(
    BallEventuallyMovesInDirectionInRegions,
    BallEventuallyStopsMovingInDirectionInRegions,
    BallAlwaysMovesInDirectionInRegions,
    BallNeverMovesInDirectionInRegions,
) = create_validation_types(BallMovesInDirectionInRegions)
