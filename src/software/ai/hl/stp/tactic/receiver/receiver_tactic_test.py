import pytest

import software.python_bindings as tbots_cpp
from proto.import_all_protos import Pass, ReceiverTactic
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.friendly_team_scored import (
    FriendlyTeamEventuallyScored,
)
from software.simulated_tests.pytest_validations.robot_at_orientation import (
    RobotEventuallyAtOrientation,
)
from software.simulated_tests.pytest_validations.robot_received_ball import (
    RobotEventuallyReceivedBall,
)
from software.simulated_tests.simulated_test_fixture import pytest_main


def calculate_ball_velocity(passer_point, receiver_point, speed):
    """Calculate ball velocity based on pass trajectory"""
    direction = receiver_point - passer_point
    if direction.length() > 0:
        direction = direction.normalize()
    return direction * speed


@pytest.mark.parametrize(
    "passer_point, receiver_point, pass_speed, robot_pos, robot_orientation, one_touch",
    [
        # Robot already at receive point
        (
            tbots_cpp.Point(0.0, 0.5),
            tbots_cpp.Point(2, 2),
            4,
            tbots_cpp.Point(2, 2),
            tbots_cpp.Angle.zero(),
            False,
        ),
        # Robot slightly off from receive point: test 1
        (
            tbots_cpp.Point(0.0, 0.4),
            tbots_cpp.Point(2, 2),
            4,
            tbots_cpp.Point(2, 1.5),
            tbots_cpp.Angle.zero(),
            False,
        ),
        # Robot slightly off from receive point: test 2
        (
            tbots_cpp.Point(0.0, 0.4),
            tbots_cpp.Point(2, 2),
            4,
            tbots_cpp.Point(2.5, 2.0),
            tbots_cpp.Angle.zero(),
            False,
        ),
        # Robot facing away from pass
        (
            tbots_cpp.Point(0.0, 0.0),
            tbots_cpp.Point(-3, 0),
            4,
            tbots_cpp.Point(-3, 0),
            tbots_cpp.Angle.half(),
            False,
        ),
        # Robot facing towards pass
        (
            tbots_cpp.Point(0.0, 0.0),
            tbots_cpp.Point(-3, 0),
            4,
            tbots_cpp.Point(-3, 0),
            tbots_cpp.Angle.zero(),
            False,
        ),
        # Robot facing towards pass speedy
        (
            tbots_cpp.Point(0.0, 0.0),
            tbots_cpp.Point(-3, 0),
            5,
            tbots_cpp.Point(-3, 0),
            tbots_cpp.Angle.zero(),
            False,
        ),
        # Sharp angles, these are only a finite set of what
        # sort of sharp angles we can achieve.
        #
        # If we are noticing issues with one-touch on the field, we should
        # add more tests here and explore more of the "one-touch" space
        #
        # TODO (#2570): re-enable when one-touch aren't flaky for these tests
        # TODO(#2909): re-enable once the robot can turn faster and hits the ball
        #
        # one touch robot on receiver point
        # (
        #     tbots_cpp.Point(2.0, 0.0),
        #     tbots_cpp.Point(3.5, 2.5),
        #     3.5,
        #     tbots_cpp.Point(3.5, 2.5),
        #     tbots_cpp.Angle.zero(),
        #     True,
        # ),
        # (
        #     tbots_cpp.Point(2.0, 0.0),
        #     tbots_cpp.Point(3.5, -2.5),
        #     3.5,
        #     tbots_cpp.Point(3.5, -2.5),
        #     tbots_cpp.Angle.zero(),
        #     True,
        # ),
        # # one touch robot away from receiver point
        # (
        #     tbots_cpp.Point(1.5, 0.0),
        #     tbots_cpp.Point(2.5, 2.5),
        #     3.5,
        #     tbots_cpp.Point(2.0, 2.5),
        #     tbots_cpp.Angle.zero(),
        #     True,
        # ),
        # (
        #     tbots_cpp.Point(1.5, 0.0),
        #     tbots_cpp.Point(2.5, -2.5),
        #     3.5,
        #     tbots_cpp.Point(2.0, -2.5),
        #     tbots_cpp.Angle.zero(),
        #     True,
        # ),
        (
            tbots_cpp.Point(4.0, 1.5),
            tbots_cpp.Point(4, -1),
            5,
            tbots_cpp.Point(4, -1),
            tbots_cpp.Angle.half(),
            True,
        ),
        # (
        #     tbots_cpp.Point(4.0, 1.5),
        #     tbots_cpp.Point(3.5, -1),
        #     5,
        #     tbots_cpp.Point(3.5, -1),
        #     tbots_cpp.Angle.zero(),
        #     True,
        # ),
        (
            tbots_cpp.Point(4.0, 1.5),
            tbots_cpp.Point(3.0, -1),
            4.5,
            tbots_cpp.Point(3.0, -1),
            tbots_cpp.Angle.half(),
            True,
        ),
        # (
        #     tbots_cpp.Point(4.0, -1.5),
        #     tbots_cpp.Point(4, 1),
        #     5,
        #     tbots_cpp.Point(4, 1),
        #     tbots_cpp.Angle.half(),
        #     True,
        # ),
        # (
        #     tbots_cpp.Point(4.0, -1.5),
        #     tbots_cpp.Point(3.5, 1),
        #     5,
        #     tbots_cpp.Point(3.5, 1),
        #     tbots_cpp.Angle.zero(),
        #     True,
        # ),
        (
            tbots_cpp.Point(4.0, -1.5),
            tbots_cpp.Point(3, 1),
            4.5,
            tbots_cpp.Point(3, 1),
            tbots_cpp.Angle.zero(),
            True,
        ),
        # (
        #     tbots_cpp.Point(3.0, 0.0),
        #     tbots_cpp.Point(2, 0),
        #     4,
        #     tbots_cpp.Point(2, 0),
        #     tbots_cpp.Angle.zero(),
        #     True,
        # ),
    ],
)
def test_receiver(
    passer_point,
    receiver_point,
    pass_speed,
    robot_pos,
    robot_orientation,
    one_touch,
    simulated_test_runner,
):
    def setup(*args):
        ball_velocity = calculate_ball_velocity(
            passer_point, receiver_point, pass_speed
        )

        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    robot_pos,
                ],
                yellow_robot_locations=[],
                ball_location=passer_point,
                ball_velocity=ball_velocity,
            )
        )

        # Necessary since pass is a python keyword
        receiver_args = {
            "pass": Pass(
                passer_point=tbots_cpp.createPointProto(passer_point),
                receiver_point=tbots_cpp.createPointProto(receiver_point),
                pass_speed_m_per_s=pass_speed,
            ),
        }

        simulated_test_runner.set_tactics(
            blue_tactics={1: ReceiverTactic(**receiver_args)}
        )

    eventually_validation_sequence_set = [
        [FriendlyTeamEventuallyScored()]
        if one_touch
        else [
            RobotEventuallyAtOrientation(
                robot_id=1,
                orientation=(passer_point - receiver_point).orientation(),
            ),
            RobotEventuallyReceivedBall(robot_id=1),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        run_till_end=False,
    )


if __name__ == "__main__":
    pytest_main(__file__)
