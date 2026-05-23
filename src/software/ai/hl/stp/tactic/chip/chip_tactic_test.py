import pytest

import software.python_bindings as tbots_cpp

from software.py_constants import ROBOT_MAX_RADIUS_METERS
from software.simulated_tests.validation.ball_is_off_ground import (
    BallIsEventuallyOffGround,
)
from software.simulated_tests.validation.ball_kicked_in_direction import (
    BallEventuallyKickedInDirection,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.import_all_protos import ChipTactic


@pytest.mark.parametrize(
    "ball_offset_from_robot, angle_to_chip_at",
    [
        # place the ball directly to the left of the robot
        (tbots_cpp.Vector(0, 0.5), tbots_cpp.Angle.zero()),
        # place the ball directly to the right of the robot
        (tbots_cpp.Vector(0, -0.5), tbots_cpp.Angle.zero()),
        # place the ball directly infront of the robot
        (tbots_cpp.Vector(0.5, 0), tbots_cpp.Angle.zero()),
        # place the ball directly behind the robot
        (tbots_cpp.Vector(-0.5, 0), tbots_cpp.Angle.zero()),
        # place the ball in the robots dribbler
        (
            tbots_cpp.Vector(ROBOT_MAX_RADIUS_METERS, 0),
            tbots_cpp.Angle.zero(),
        ),
        # Repeat the same tests but kick in the opposite direction
        # place the ball directly to the left of the robot
        (tbots_cpp.Vector(0, 0.5), tbots_cpp.Angle.half()),
        # place the ball directly to the right of the robot
        (tbots_cpp.Vector(0, -0.5), tbots_cpp.Angle.half()),
        # place the ball directly infront of the robot
        (tbots_cpp.Vector(0.5, 0), tbots_cpp.Angle.half()),
        # place the ball directly behind the robot
        (tbots_cpp.Vector(-0.5, 0), tbots_cpp.Angle.half()),
        # place the ball in the robots dribbler
        (
            tbots_cpp.Vector(ROBOT_MAX_RADIUS_METERS, 0),
            tbots_cpp.Angle.zero(),
        ),
    ],
)
def test_chip(ball_offset_from_robot, angle_to_chip_at, simulated_test_runner):
    robot_position = tbots_cpp.Point(0, 0)
    ball_position = robot_position + ball_offset_from_robot

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    robot_position,
                ],
                yellow_robot_locations=[tbots_cpp.Point(4, 0)],
                ball_location=ball_position,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: ChipTactic(
                    chip_origin=tbots_cpp.createPointProto(ball_position),
                    chip_direction=tbots_cpp.createAngleProto(angle_to_chip_at),
                    chip_distance_meters=2.0,
                )
            }
        )

    eventually_validations = [
        [
            BallEventuallyKickedInDirection(angle_to_chip_at),
        ],
        [
            BallIsEventuallyOffGround(),
        ],
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
    )


if __name__ == "__main__":
    pytest_main(__file__)
