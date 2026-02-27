import pytest
import math

import software.python_bindings as tbots_cpp
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state


@pytest.mark.parametrize(
    "ball_offset_from_robot, angle_to_kick_at",
    [
        # place the ball directly to the left of the robot
        (tbots_cpp.Vector(0, 0.5), 0),
        # place the ball directly to the right of the robot
        (tbots_cpp.Vector(0, -0.5), 0),
        # place the ball directly infront of the robot
        (tbots_cpp.Vector(0.5, 0), 0),
        # place the ball directly behind the robot
        (tbots_cpp.Vector(-0.5, 0), 0),
        # place the ball in the robots dribbler
        (tbots_cpp.Vector(0.1, 0), 0),
        # place the ball directly to the right of the robot, kick backwards
        (tbots_cpp.Vector(0, -0.5), math.pi),
        # place the ball directly infront of the robot, kick backwards
        (tbots_cpp.Vector(0.5, 0), math.pi),
        # place the ball in the robots dribbler
        (tbots_cpp.Vector(0.1, 0), 0),
    ],
)
def test_pivot_kick(ball_offset_from_robot, angle_to_kick_at, simulated_test_runner):
    robot_position = tbots_cpp.Point(0, 0)
    ball_position = robot_position + ball_offset_from_robot

    # Setup World
    def setup(*args):
        robot_position = tbots_cpp.Point(0, 0)
        ball_position = robot_position + ball_offset_from_robot
        simulated_test_runner.set_worldState(
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

        blue_play = Play()
        blue_play.name = PlayName.AssignedTacticsPlay
        simulated_test_runner.set_play(blue_play, True)

        params = AssignedTacticPlayControlParams()
        params.assigned_tactics[1].pivot_kick.CopyFrom(
            PivotKickTactic(
                kick_origin=tbots_cpp.createPointProto(ball_position),
                kick_direction=tbots_cpp.createAngleProto(
                    tbots_cpp.Angle.fromRadians(angle_to_kick_at)
                ),
                auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=5.0),
            )
        )
        simulated_test_runner.set_tactics(params, True)

        params = AssignedTacticPlayControlParams()
        simulated_test_runner.set_tactics(params, False)

    kick_direction_vector = tbots_cpp.Vector(1, 0).rotate(
        tbots_cpp.Angle.fromRadians(angle_to_kick_at)
    )
    kick_target = ball_position + kick_direction_vector * 3

    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(ball_position, 0.3)]),
            BallEventuallyEntersRegion(regions=[tbots_cpp.Circle(kick_target, 1.0)]),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=3,
    )


if __name__ == "__main__":
    pytest_main(__file__)
