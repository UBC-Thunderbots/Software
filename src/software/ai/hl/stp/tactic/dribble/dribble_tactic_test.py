import pytest

import software.python_bindings as tbots_cpp
from proto.import_all_protos import DribbleTactic
from software.simulated_tests.pytest_validations.robot_enters_region import *
from software.simulated_tests.pytest_validations.ball_enters_region import *
from software.simulated_tests.pytest_validations.robot_received_ball import *
from software.simulated_tests.pytest_validations.robot_at_orientation import *
from software.simulated_tests.pytest_validations.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state


def get_enemy_robot_positions():
    field = tbots_cpp.Field.createSSLDivisionBField()
    return [
        tbots_cpp.Point(1, 0),
        tbots_cpp.Point(1, 2.5),
        tbots_cpp.Point(1, -2.5),
        field.enemyGoalCenter(),
        field.enemyDefenseArea().negXNegYCorner(),
        field.enemyDefenseArea().negXPosYCorner(),
    ]


def test_intercept_ball_behind_enemy_robot(simulated_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    initial_position,
                ],
                yellow_robot_locations=get_enemy_robot_positions(),
                ball_location=tbots_cpp.Point(3, -2),
                ball_velocity=tbots_cpp.Vector(-0.5, 1),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: DribbleTactic(
                    dribble_destination=tbots_cpp.createPointProto(
                        tbots_cpp.Point(0, 0)
                    ),
                    final_dribble_orientation=tbots_cpp.createAngleProto(
                        tbots_cpp.Angle.zero()
                    ),
                    allow_excessive_dribbling=False,
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyReceivedBall(robot_id=1, tolerance=0.05),
            # TODO (#2588): validate that robot keeps possession
            # TODO (#2514): tune dribbling and re-enable
            # Robot not excessively dribbling
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


def test_stopped_ball(simulated_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(3, 3),
                    initial_position,
                ],
                yellow_robot_locations=get_enemy_robot_positions(),
                ball_location=tbots_cpp.Point(-1, 1.5),
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: DribbleTactic(
                    dribble_destination=tbots_cpp.createPointProto(
                        tbots_cpp.Point(0, 0)
                    ),
                    final_dribble_orientation=tbots_cpp.createAngleProto(
                        tbots_cpp.Angle.zero()
                    ),
                    allow_excessive_dribbling=False,
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyReceivedBall(robot_id=1, tolerance=0.05),
            # TODO (#2514): tune dribbling and re-enable
            # Robot not excessively dribbling
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


def test_ball_bounce_off_of_enemy_robot(simulated_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(3, 3),
                    initial_position,
                ],
                yellow_robot_locations=get_enemy_robot_positions(),
                ball_location=tbots_cpp.Point(0, 0),
                ball_velocity=tbots_cpp.Vector(2.5, 0),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: DribbleTactic(
                    dribble_destination=tbots_cpp.createPointProto(
                        tbots_cpp.Point(0, 0)
                    ),
                    final_dribble_orientation=tbots_cpp.createAngleProto(
                        tbots_cpp.Angle.zero()
                    ),
                    allow_excessive_dribbling=False,
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyReceivedBall(robot_id=1, tolerance=0.05),
            # TODO (#2514): tune dribbling and re-enable
            # Robot not excessively dribbling
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


@pytest.mark.parametrize(
    "dribble_destination, dribble_orientation, ball_pos, ball_vel",
    [
        (
            tbots_cpp.Point(-3, 1),
            None,
            tbots_cpp.Point(3, -2),
            tbots_cpp.Vector(-1, 2),
        ),
        (
            None,
            tbots_cpp.Angle.quarter(),
            tbots_cpp.Point(3, -2),
            tbots_cpp.Vector(-1, 2),
        ),
        (
            tbots_cpp.Point(-1, 2),
            tbots_cpp.Angle.zero(),
            tbots_cpp.Point(1, 0),
            tbots_cpp.Vector(1, 2),
        ),
    ],
)
def test_moving_ball_dribble(
    dribble_destination, dribble_orientation, ball_pos, ball_vel, simulated_test_runner
):
    initial_position = tbots_cpp.Point(-3, 1.5)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    initial_position,
                ],
                yellow_robot_locations=get_enemy_robot_positions(),
                ball_location=ball_pos,
                ball_velocity=ball_vel,
            )
        )

        dribble_params = DribbleTactic(
            allow_excessive_dribbling=False,
        )
        if dribble_destination:
            dribble_params.dribble_destination.CopyFrom(
                tbots_cpp.createPointProto(dribble_destination)
            )
        if dribble_orientation:
            dribble_params.final_dribble_orientation.CopyFrom(
                tbots_cpp.createAngleProto(dribble_orientation)
            )

        simulated_test_runner.set_tactics(blue_tactics={1: dribble_params})

    validations = [RobotEventuallyReceivedBall(robot_id=1, tolerance=0.05)]

    if dribble_destination:
        validations.append(
            BallEventuallyEntersRegion(
                regions=[tbots_cpp.Circle(dribble_destination, 0.3)]
            )
            # TODO (#2514): tune dribbling and re-enable
            # Robot not excessively dribbling
        )

    if dribble_orientation:
        validations.append(
            RobotEventuallyAtOrientation(
                robot_id=1, orientation=dribble_orientation, threshold=0.1
            )
        )

    eventually_validation_sequence_set = [validations]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=15,
    )


def test_dribble_with_excessive_dribbling(simulated_test_runner):
    dribble_destination = tbots_cpp.Point(3, 2)
    initial_position = tbots_cpp.Point(4.5, -3.0)
    dribble_orientation = tbots_cpp.Angle.half()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    initial_position,
                ],
                yellow_robot_locations=get_enemy_robot_positions(),
                ball_location=tbots_cpp.Point(4.2, -2.5),
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: DribbleTactic(
                    dribble_destination=tbots_cpp.createPointProto(dribble_destination),
                    final_dribble_orientation=tbots_cpp.createAngleProto(
                        dribble_orientation
                    ),
                    allow_excessive_dribbling=True,
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            RobotEventuallyReceivedBall(robot_id=1, tolerance=0.05),
            BallEventuallyEntersRegion(
                regions=[tbots_cpp.Circle(dribble_destination, 0.3)]
            ),
            RobotEventuallyAtOrientation(
                robot_id=1, orientation=dribble_orientation, threshold=0.1
            ),
            # TODO (#2514): tune dribbling and re-enable
            # Robot eventually excessively dribbling
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=12,
    )


if __name__ == "__main__":
    pytest_main(__file__)
