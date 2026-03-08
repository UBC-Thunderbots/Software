import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.pytest_validations.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class RobotReceivedBall(Validation):
    """Checks if a specific robot has received the ball (ball is near dribbler)"""

    def __init__(self, robot_id, tolerance=0.02):
        """Constructs the validation object

        :param robot_id: The ID of the robot to check
        :param tolerance: The tolerance for when we check if the robot has the ball
        """
        self.robot_id = robot_id
        self.tolerance = tolerance

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the specific robot has received the ball

        :param world: The world msg to validate
        :return: FAILING when the robot does not have the ball
                 PASSING when the robot has the ball
        """
        ball_position = tbots_cpp.createPoint(world.ball.current_state.global_position)
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                if tbots_cpp.Robot(robot).isNearDribbler(ball_position, self.tolerance):
                    return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Highlights the dribbler area of the robot"""
        # TODO (#3637): create better validation geometry
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                return create_validation_geometry(
                    [tbots_cpp.Robot(robot).dribblerArea()]
                )
        return create_validation_geometry()

    @override
    def __repr__(self):
        return f"Check that robot {self.robot_id} has received the ball"


(
    RobotEventuallyReceivedBall,
    RobotEventuallyDidNotReceiveBall,
    RobotAlwaysHasBall,
    RobotNeverHasBall,
) = create_validation_types(RobotReceivedBall)
