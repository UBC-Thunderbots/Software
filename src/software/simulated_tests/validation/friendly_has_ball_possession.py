import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class FriendlyHasBallPossession(Validation):
    """Checks if friendly robot has possession of the ball."""

    def __init__(self, robot_id=None, tolerance=0.01):
        """Constructs the validation object

        :param robot_id: The robot id to check, None to check for any friendly robot
        :param tolerance: The tolerance for when we check if the robot has the ball
        """
        self.robot_id = robot_id
        self.tolerance = tolerance

    @override
    def get_validation_status(self, world, simulator_state=None) -> ValidationStatus:
        """Checks if friendly robot has possession of the ball

        :param world: The world msg to validate
        :return: FAILING when friendly robot does not have possession of the ball
                 PASSING when friendly robot has possession of the ball
        """
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if self.robot_id is not None and robot.id != self.robot_id:
                continue
            if tbots_cpp.Robot(robot).isNearDribbler(ball_position, self.tolerance):
                return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """(override) highlights the dribbler area of the robots"""
        return create_validation_geometry(
            [
                tbots_cpp.Robot(robot).dribblerArea()
                for robot in world.friendly_team.team_robots
            ]
        )

    @override
    def __repr__(self):
        return (
            "Check that the friendly team has possession of the ball"
            if self.robot_id is None
            else f"Check that friendly robot {self.robot_id} has possession of the ball"
        )


(
    FriendlyEventuallyHasBallPossession,
    FriendlyEventuallyLosesBallPossession,
    FriendlyAlwaysHasBallPossession,
    FriendlyNeverHasBallPossession,
) = create_validation_types(FriendlyHasBallPossession)
