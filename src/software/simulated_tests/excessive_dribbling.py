import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class ExcessivelyDribbling(Validation):

    """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m."""

    def __init__(self):
        self.continous_dribbling_start_point = None

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m.

        :param world: The world msg to validate
        :returns: FAILING when the robot is excessively dribbling
                  PASSING when the robot is not excessively dribbling
        """
        ball_position = tbots.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if not tbots.Robot(robot).isNearDribbler(ball_position, 0.01):
                self.continous_dribbling_start_point = ball_position
            elif (
                ball_position - (self.continous_dribbling_start_point or ball_position)
            ).length() > 1.0:
                return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) Shows the max allowed dribbling circle
        """
        return create_validation_geometry(
            [tbots.Circle(self.continous_dribbling_start_point, 1.0)]
            if self.continous_dribbling_start_point is not None
            else []
        )

    def __repr__(self):
        return "Check that the dribbling robot has not dribbled for more than 1m"


(
    EventuallyStopExcessivelyDribbling,
    EventuallyExcessivelyDribbling,
    NeverExcessivelyDribbling,
    AlwaysExcessivelyDribbling,
) = create_validation_types(ExcessivelyDribbling)
