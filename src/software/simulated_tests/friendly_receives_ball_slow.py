import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class FriendlyReceivesBallSlow(Validation):

    """

    """
    def __init__(self, robot_id, max_receive_speed):
        self.robot_id = robot_id
        self.max_receive_speed = max_receive_speed

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if any friendly robot has possession of the ball

        :param world: The world msg to validate
        :returns: FAILING when no friendly robot has possession of the ball
                  PASSING when any friendly robot has possession of the ball
        """
        ball_position = tbots.createPoint(world.ball.current_state.global_position)
        ball_velocity = tbots.createVector(world.ball.current_state.global_velocity)
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                if tbots.Robot(robot).isNearDribbler(ball_position, 0.01):
                    if ball_velocity.length() - self.max_receive_speed > 0.2:
                        return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) highlights the dribbler area of the robots
        """
        return create_validation_geometry(
            [
                tbots.Robot(robot).dribblerArea()
                for robot in world.friendly_team.team_robots
            ]
        )

    def __repr__(self):
        return f"Check that robot {self.robot_id} is receiving the ball slowly"


(
    FriendlyEventuallyReceivesBallSlow,
    FriendlyEventuallyLosesBallSlow,
    FriendlyAlwaysReceivesBallSlow,
    FriendlyNeverReceivesBallSlow,
) = create_validation_types(FriendlyReceivesBallSlow)
