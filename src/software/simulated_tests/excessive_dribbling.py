import software.geom.geometry as tbots_geom
import software.world.world as tbots_world
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class ExcessivelyDribbling(Validation):

    """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m."""

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m.

        :param world: The world msg to validate
        :returns: FAILING when the robot is excessively dribbling
                  PASSING when the robot is not excessively dribbling
        """
        ball_position = tbots_geom.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if not tbots_world.Robot(robot).isNearDribbler(ball_position, 0.01):
                self.continous_dribbling_start_point = ball_position
            elif (ball_position - self.continous_dribbling_start_point).length() > 1.0:
                return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        # TODO: visualize
        return create_validation_geometry([])

    def __repr__(self):
        return "Check that the friendly team has possession of the ball"


(
    EventuallyExcessivelyDribbling,
    EventuallyStopExcessivelyDribbling,
    AlwaysExcessivelyDribbling,
    NeverExcessivelyDribbling,
) = create_validation_types(ExcessivelyDribbling)
