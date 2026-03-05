import pytest
import software.python_bindings as tbots_cpp
from software.py_constants import ROBOT_MAX_RADIUS_METERS

from proto.import_all_protos import AssignedTacticPlayControlParams, KickTactic
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.ball_enters_region import (
    BallEventuallyEntersRegion,
)
from software.simulated_tests.pytest_validations.ball_kicked_in_direction import (
    BallEventuallyKickedInDirection,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "ball_offset_from_robot, angle_to_kick_at",
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
        (tbots_cpp.Vector(ROBOT_MAX_RADIUS_METERS, 0), tbots_cpp.Angle.zero()),
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
        (tbots_cpp.Vector(ROBOT_MAX_RADIUS_METERS, 0), tbots_cpp.Angle.zero()),
    ],
)
def test_kick(ball_offset_from_robot, angle_to_kick_at, simulated_test_runner):
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
        params.assigned_tactics[1].kick.CopyFrom(
            KickTactic(
                kick_origin=tbots_cpp.createPointProto(ball_position),
                kick_direction=tbots_cpp.createAngleProto(angle_to_kick_at),
                kick_speed_meters_per_second=5.0,
            )
        )
        simulated_test_runner.set_tactics(params, is_friendly=True)

        params = AssignedTacticPlayControlParams()
        simulated_test_runner.set_tactics(params, is_friendly=False)

    kick_direction_vector = tbots_cpp.Vector(1, 0).rotate(angle_to_kick_at)
    kick_target = ball_position + kick_direction_vector * 3

    eventually_validation_sequence_set = [
        [
            BallEventuallyEntersRegion(regions=[tbots_cpp.Circle(kick_target, 0.5)]),
            BallEventuallyKickedInDirection(kick_direction=angle_to_kick_at),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=5,
    )


if __name__ == "__main__":
    pytest_main(__file__)
