import software.geom.geometry as tbots_geom
import software.world.world as tbots_world
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class FriendlyHasBallPossession(Validation):

    """Checks if any friendly robot has possession of the ball."""

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot has possession of the ball

        :param world: The world msg to validate
        :returns: FAILING when no friendly robot has possession of the ball
                  PASSING when any friendly robot has possession of the ball
        """
        ball_position = tbots_geom.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if tbots_world.Robot(robot).isNearDribbler(ball_position, 0.01):
                return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """override"""
        # TODO: visualize
        return create_validation_geometry([])

    def __repr__(self):
        return "Check that the friendly team has possession of the ball"


(
    FriendlyEventuallyHasBallPossession,
    FriendlyEventuallyLosesBallPossession,
    FriendlyAlwaysHasBallPossession,
    FriendlyNeverHasBallPossession,
) = create_validation_types(FriendlyHasBallPossession)
