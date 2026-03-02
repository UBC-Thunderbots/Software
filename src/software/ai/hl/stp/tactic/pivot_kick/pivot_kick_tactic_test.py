import pytest
import software.python_bindings as tbots_cpp

from proto.message_translation.tbots_protobuf import create_world_state
from proto.import_all_protos import *
from software.simulated_tests.pytest_validations.ball_enters_region import (
    BallEventuallyEntersRegion,
)
from software.simulated_tests.pytest_validations.ball_kicked import BallEventuallyKicked
from software.simulated_tests.pytest_validations.robot_enters_region import (
    RobotEventuallyEntersRegion,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "ball_offset_from_robot, angle_to_kick_at",
    [
        # TODO (#2859): Flaky, the robot does not dribble far enough into the ball
        # # place the ball directly to the left of the robot
        # (tbots_cpp.Vector(0, 0.5), tbots_cpp.Angle.zero()),
        # # place the ball directly to the right of the robot
        # (tbots_cpp.Vector(0, -0.5), tbots_cpp.Angle.zero()),
        # # place the ball directly infront of the robot
        # (tbots_cpp.Vector(0.5, 0), tbots_cpp.Angle.zero()),
        # # place the ball directly behind the robot
        # (tbots_cpp.Vector(-0.5, 0), tbots_cpp.Angle.zero()),
        # # place the ball in the robots dribbler
        # (tbots_cpp.Vector(0.1, 0), tbots_cpp.Angle.zero()),
        # # place the ball directly to the left of the robot, kick left
        # (tbots_cpp.Vector(0, 0.5), tbots_cpp.Angle.half()),
        # # place the ball directly behind the robot, kick left
        # (tbots_cpp.Vector(-0.5, 0), tbots_cpp.Angle.half()),
    ],
)
def test_pivot_kick(ball_offset_from_robot, angle_to_kick_at, simulated_test_runner):
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

        params = AssignedTacticPlayControlParams()
        params.assigned_tactics[1].pivot_kick.CopyFrom(
            PivotKickTactic(
                kick_origin=tbots_cpp.createPointProto(ball_position),
                kick_direction=tbots_cpp.createAngleProto(angle_to_kick_at),
                auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=5.0),
            )
        )
        simulated_test_runner.set_tactics(params, is_friendly=True)

        params = AssignedTacticPlayControlParams()
        simulated_test_runner.set_tactics(params, is_friendly=False)

    kick_direction_vector = tbots_cpp.Vector(1, 0).rotate(angle_to_kick_at)
    kick_target = ball_position + kick_direction_vector * 3

    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(ball_position, 0.5)]),
            BallEventuallyEntersRegion(regions=[tbots_cpp.Circle(kick_target, 1.0)]),
            BallEventuallyKicked(kick_direction=angle_to_kick_at),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
