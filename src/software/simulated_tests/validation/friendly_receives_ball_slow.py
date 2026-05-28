import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation.validation import (
    create_validation_types,
)
from software.simulated_tests.validation.friendly_has_ball_possession import (
    FriendlyHasBallPossession,
)
from typing import override


class FriendlyReceivesBallSlow(FriendlyHasBallPossession):
    """Checks if a receiver receives the ball below a certain speed"""

    def __init__(self, robot_id, max_receive_speed):
        """Constructs the validation to check the given robot ID for receiving the ball
        at the given speed

        :param robot_id: the robot id to check
        :param max_receive_speed: the max speed the ball should be received at
        """
        super().__init__(robot_id=robot_id)
        self.max_receive_speed = max_receive_speed

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the specified robot receives the ball too fast

        :param world: The world msg to validate
        :return: FAILING if the ball is near the robot's dribbler at a speed higher
                         than the max receive speed
                 PASSING if the ball is not near the dribbler, or if it is near
                         the dribbler at a speed slower than the max
        """
        if super().get_validation_status(world) == ValidationStatus.PASSING:
            ball_velocity = tbots_cpp.createVector(
                world.ball.current_state.global_velocity
            )
            if ball_velocity.length() - self.max_receive_speed > 0.2:
                return ValidationStatus.FAILING
        return ValidationStatus.PASSING

    @override
    def __repr__(self):
        return f"Check that robot {self.robot_id} is receiving the ball slowly"


(
    FriendlyEventuallyReceivesBallSlow,
    FriendlyEventuallyLosesBallSlow,
    FriendlyAlwaysReceivesBallSlow,
    FriendlyNeverReceivesBallSlow,
) = create_validation_types(FriendlyReceivesBallSlow)
