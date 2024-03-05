import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallKicked(Validation):

    """Checks if the ball is kicked by a robot."""

    BALL_MOVED_M = 0.02

    def __init__(self, threshold=0.01):
        """
        :param threshold: The distance from the ball to consider a robot touching it
        """
        self.kicker_robot = None
        self.kick_position = None
        self.threshold = threshold

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball is kicked by a robot

        :param world: The world msg to validate
        :returns: FAILING when the ball is not kicked
                  PASSING when the ball is kicked
        """
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)
        if not self.kick_position:
            self.kick_position = ball_position
        # quick passing check
        if self.kicker_robot is not None and self.kick_position and (ball_position - self.kick_position).length() > BALL_MOVED_M:
            return ValidationStatus.PASSING
        # main check loop
        for robot in world.friendly_team.team_robots:
            if tbots_cpp.Robot(robot).isNearDribbler(ball_position, self.threshold):
                if self.kicker_robot is None:
                    self.kicker_robot = robot
        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry(
            [tbots_cpp.Field.createSSLDivisionBField().fieldLines()]
        )

    def __repr__(self):
        return "Checking that the ball is kicked"


(
    BallEventuallyNotKicked,
    BallEventuallyKicked,
    BallAlwaysNotKicked,
    BallAlwaysKicked,
) = create_validation_types(BallKicked)
