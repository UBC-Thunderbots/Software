import pytest
import software.python_bindings as tbots_cpp
from software.py_constants import DIST_TO_FRONT_OF_ROBOT_METERS, ROBOT_MAX_RADIUS_METERS

from proto.import_all_protos import DribbleTactic
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.validation.ball_enters_region import (
    BallAlwaysStaysInRegion,
    BallEventuallyEntersRegion,
)
from software.simulated_tests.validation.excessive_dribbling import (
    EventuallyStartsExcessivelyDribbling,
    NeverExcessivelyDribbles,
)
from software.simulated_tests.validation.robot_at_orientation import (
    RobotEventuallyAtOrientation,
)
from software.simulated_tests.validation.robot_received_ball import (
    RobotEventuallyReceivedBall,
)
from software.simulated_tests.validation.delay_validation import (
    DelayValidation,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


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


@pytest.mark.parametrize(
    "initial_pos, dribble_dest, dribble_orientation, ball_pos, ball_vel",
    [
        # test_intercept_ball_behind_enemy_robot
        (
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(0, 0),
            None,
            tbots_cpp.Point(3, -2),
            tbots_cpp.Vector(-0.5, 1),
        ),
        # test_stopped_ball
        (
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(0, 0),
            None,
            tbots_cpp.Point(-1, 1.5),
            tbots_cpp.Vector(0, 0),
        ),
        # test_ball_bounce_off_of_enemy_robot
        (
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(0, 0),
            None,
            tbots_cpp.Point(0, 0),
            tbots_cpp.Vector(2.5, 0),
        ),
        # test_moving_ball_dribble_dest
        (
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(-3, 1),
            None,
            tbots_cpp.Point(3, -2),
            tbots_cpp.Vector(-1, 2),
        ),
        # test_moving_ball_dribble_orientation
        (
            tbots_cpp.Point(-3, 1.5),
            None,
            tbots_cpp.Angle.quarter(),
            tbots_cpp.Point(3, -2),
            tbots_cpp.Vector(-1, 2),
        ),
        # test_moving_ball_dribble_dest_and_orientation
        (
            tbots_cpp.Point(-2, 1.5),
            tbots_cpp.Point(-1, 2),
            tbots_cpp.Angle.zero(),
            tbots_cpp.Point(1, 0),
            tbots_cpp.Vector(1, 2),
        ),
        # test_dribble_dest_and_orientation_around_rectangle
        (
            tbots_cpp.Point(3, -3),
            tbots_cpp.Point(4, 2.5),
            tbots_cpp.Angle.half(),
            tbots_cpp.Point(4, -2.5),
            tbots_cpp.Vector(0, 0),
        ),
        # test_steal_ball_from_behind_enemy 1
        (
            tbots_cpp.Point(-2, 2.5),
            tbots_cpp.Point(0, 2),
            tbots_cpp.Angle.zero(),
            tbots_cpp.Point(1 + DIST_TO_FRONT_OF_ROBOT_METERS, 2.5),
            tbots_cpp.Vector(0, 0),
        ),
        # test_steal_ball_from_behind_enemy 2
        (
            tbots_cpp.Point(3.5, 2.5),
            tbots_cpp.Point(0, 2),
            tbots_cpp.Angle.zero(),
            tbots_cpp.Point(1 + DIST_TO_FRONT_OF_ROBOT_METERS, 2.5),
            tbots_cpp.Vector(0, 0),
        ),
        # test_steal_ball_from_behind_enemy 3
        (
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(0, 2),
            tbots_cpp.Angle.zero(),
            tbots_cpp.Point(1 + DIST_TO_FRONT_OF_ROBOT_METERS, 2.5),
            tbots_cpp.Vector(0, 0),
        ),
    ],
)
def test_dribble(
    initial_pos,
    dribble_dest,
    dribble_orientation,
    ball_pos,
    ball_vel,
    simulated_test_runner,
):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    initial_pos,
                ],
                yellow_robot_locations=get_enemy_robot_positions(),
                ball_location=ball_pos,
                ball_velocity=ball_vel,
            )
        )

        dribble_params = DribbleTactic(
            allow_excessive_dribbling=False,
        )
        if dribble_dest:
            dribble_params.dribble_destination.CopyFrom(
                tbots_cpp.createPointProto(dribble_dest)
            )
        if dribble_orientation:
            dribble_params.final_dribble_orientation.CopyFrom(
                tbots_cpp.createAngleProto(dribble_orientation)
            )

        simulated_test_runner.set_tactics(blue_tactics={1: dribble_params})

    eventually_validations = [[RobotEventuallyReceivedBall(robot_id=1)]]

    if dribble_dest:
        eventually_validations[0].append(
            BallEventuallyEntersRegion([tbots_cpp.Circle(dribble_dest, 0.3)])
        )

    if dribble_orientation:
        eventually_validations[0].append(
            RobotEventuallyAtOrientation(1, dribble_orientation)
        )

    eventually_validations[0].append(
        DelayValidation(delay_s=2, validation=RobotEventuallyReceivedBall(1))
    )

    # TODO (#2514): tune dribbling and re-enable
    # Robot always not excessively dribbling

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        test_timeout_s=25,
        run_till_end=False,
    )


@pytest.mark.parametrize(
    "initial_ball_location,dribble_destination,final_dribble_orientation, should_excessively_dribble, blue_robot_location",
    [
        # The following tests check that DribbleTactic does not false trigger for distances within regulations
        # Dribble Destination for the ball < 1.0 from its starting position
        (
            tbots_cpp.Point(0.5, 0),
            tbots_cpp.Point(1.02, 0),
            tbots_cpp.Angle(),
            False,
            tbots_cpp.Point(0, 1),
        ),
        # Dribble Testing diagonally
        (
            tbots_cpp.Point(0.25, 0.25),
            tbots_cpp.Point(0.80, 0.50),
            tbots_cpp.Angle.fromRadians(50),
            False,
            tbots_cpp.Point(0, 1),
        ),
        # Boundary Testing, because of the autoref implementation (initial of position Bot to final of Ball),
        # a conservative max dribble distance (0.95 m) is used
        # Test vertical dribbling
        (
            tbots_cpp.Point(0.01, 0),
            tbots_cpp.Point(0.96, 0),
            tbots_cpp.Angle(),
            False,
            tbots_cpp.Point(0, 1),
        ),
        # Test horizontal dribbling
        (
            tbots_cpp.Point(1, 1.5),
            tbots_cpp.Point(1.95, 1.5),
            tbots_cpp.Angle(),
            False,
            tbots_cpp.Point(0, 1),
        ),
        # Test bot and ball in same position
        (
            tbots_cpp.Point(0, 1),
            tbots_cpp.Point(0.95, 1),
            tbots_cpp.Angle(),
            False,
            tbots_cpp.Point(0, 1),
        ),
        # The following tests check that DribbleTactic correctly triggers for distances outside regulation
        # Dribble Destination for the ball > 1.0 from its starting position
        (
            tbots_cpp.Point(0, 2),
            tbots_cpp.Point(0, 0.5),
            tbots_cpp.Angle(),
            True,
            tbots_cpp.Point(0, 0),
        ),
        # Dribble Testing diagonally
        (
            tbots_cpp.Point(0.1, 1.1),
            tbots_cpp.Point(1.1, 0.1),
            tbots_cpp.Angle.fromRadians(50),
            True,
            tbots_cpp.Point(0, 0),
        ),
        # Boundary Testing, due to the conservative implementation a dribble distance of 1 m should fail
        # Test Vertical Dribbling
        (
            tbots_cpp.Point(0, 1),
            tbots_cpp.Point(0, 2),
            tbots_cpp.Angle(),
            True,
            tbots_cpp.Point(0, 0),
        ),
        # Test Horizontal Dribbling
        (
            tbots_cpp.Point(1, 2),
            tbots_cpp.Point(0, 2),
            tbots_cpp.Angle(),
            True,
            tbots_cpp.Point(0, 0),
        ),
        # Test Diagonal Dribbling
        (
            tbots_cpp.Point(0, 1),
            tbots_cpp.Point(0.6, 1.8),
            tbots_cpp.Angle(),
            True,
            tbots_cpp.Point(0, 0),
        ),
        # Test robot and ball at same position (affects dribbling orientation and therefore perceived dribble distance)
        (
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0, 1),
            tbots_cpp.Angle(),
            True,
            tbots_cpp.Point(0, 0),
        ),
        (
            tbots_cpp.Point(0.0, 0.01),
            tbots_cpp.Point(0.81, 0.61),
            tbots_cpp.Angle(),
            True,
            tbots_cpp.Point(0, 0),
        ),
    ],
)
def test_excessive_dribbling_without_enemies(
    initial_ball_location,
    dribble_destination,
    final_dribble_orientation,
    should_excessively_dribble,
    simulated_test_runner,
    blue_robot_location,
):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[blue_robot_location],
                yellow_robot_locations=[],
                ball_location=initial_ball_location,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0:
                DribbleTactic(
                    dribble_destination=tbots_cpp.createPointProto(dribble_destination),
                    final_dribble_orientation=tbots_cpp.createAngleProto(
                        final_dribble_orientation
                    ),
                    allow_excessive_dribbling=True,
                )
            }
        )

    if should_excessively_dribble:
        # Always and Eventually validation sets for excessive dribbling
        always_validation_sequence_set = [[]]
        eventually_validation_sequence_set = [[EventuallyStartsExcessivelyDribbling()]]
    else:
        # Always and Eventually validation sets for not excessive dribbling
        always_validation_sequence_set = [[NeverExcessivelyDribbles()]]
        eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
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
            RobotEventuallyReceivedBall(1),
            BallEventuallyEntersRegion([tbots_cpp.Circle(dribble_destination, 0.3)]),
            RobotEventuallyAtOrientation(1, dribble_orientation),
        ],
        [EventuallyStartsExcessivelyDribbling()],
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        test_timeout_s=10,
    )


def test_run_into_enemy_robot_knock_ball_away(
    simulated_test_runner,
):
    initial_position = tbots_cpp.Point(-2, 1.5)
    dribble_destination = tbots_cpp.Point(-1, 2)
    dribble_orientation = tbots_cpp.Angle.half()
    ball_pos = tbots_cpp.Point(2, -2)
    ball_vel = tbots_cpp.Vector(2, 4)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    initial_position,
                ],
                yellow_robot_locations=get_enemy_robot_positions()
                + [tbots_cpp.Point(1, 1.1)],
                ball_location=ball_pos,
                ball_velocity=ball_vel,
            )
        )

        dribble_params = DribbleTactic(
            allow_excessive_dribbling=False,
            dribble_destination=tbots_cpp.createPointProto(dribble_destination),
            final_dribble_orientation=tbots_cpp.createAngleProto(dribble_orientation),
        )

        simulated_test_runner.set_tactics(blue_tactics={1: dribble_params})

    eventually_validations = [
        [
            RobotEventuallyReceivedBall(1),
            BallEventuallyEntersRegion([tbots_cpp.Circle(dribble_destination, 0.3)]),
            RobotEventuallyAtOrientation(1, dribble_orientation),
            DelayValidation(delay_s=2, validation=RobotEventuallyReceivedBall(1)),
        ]
    ]

    # TODO (#2514): tune dribbling and re-enable
    # Robot always not excessively dribbling

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        test_timeout_s=10,
    )


def test_robot_not_bumping_ball_when_turning(
    simulated_test_runner,
):
    # The ball is placed right behind the friendly robot. Verify that the robot
    # does not bump the ball away when turning around to dribble it.
    robot_location = tbots_cpp.Point(-1, 0)
    ball_location = robot_location + tbots_cpp.Vector(ROBOT_MAX_RADIUS_METERS, 0)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[tbots_cpp.Point(-3, 2.5), robot_location],
                blue_robot_orientations=[
                    tbots_cpp.Angle.zero(),
                    tbots_cpp.Angle.half(),
                ],
                yellow_robot_locations=get_enemy_robot_positions()
                + [tbots_cpp.Point(1, 1.1)],
                ball_location=ball_location,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        dribble_params = DribbleTactic()
        simulated_test_runner.set_tactics(blue_tactics={1: dribble_params})

    eventually_validations = [
        [
            RobotEventuallyReceivedBall(1),
        ]
    ]

    # TODO (#2514): tune dribbling and re-enable
    # Robot always not excessively dribbling
    always_validations = [
        [BallAlwaysStaysInRegion([tbots_cpp.Circle(ball_location, 0.05)])]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        inv_always_validation_sequence_set=always_validations,
        ag_always_validation_sequence_set=always_validations,
    )


if __name__ == "__main__":
    pytest_main(__file__)
