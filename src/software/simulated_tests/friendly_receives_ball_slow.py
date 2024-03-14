import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)

class FriendlyReceivesBallSlow(Validation):

    """
    Checks if a receiver receives the ball below a certain speed
    """

    def __init__(self, robot_id, max_receive_speed):
        """
        Constructs the validation to check the given robot ID for receiving the ball
        at the given speed
        :param robot_id: the robot id to check
        :param max_receive_speed: the max speed the ball should be received at
        """
        self.robot_id = robot_id
        self.max_receive_speed = max_receive_speed

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the specified robot receives the ball too fast

        :param world: The world msg to validate
        :returns: FAILING if the ball is near the robot's dribbler at a speed higher
                    than the max receive speed
                  PASSING if the ball is not near the dribbler, or if it is near
                    the dribbler at a speed slower than the max
        """
        ball_position = tbots.createPoint(world.ball.current_state.global_position)
        ball_velocity = tbots.createVector(world.ball.current_state.global_velocity)
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                # tolerance is set to 0.1 to check the speed before the ball touches the robot
                if tbots.Robot(robot).isNearDribbler(ball_position):
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
