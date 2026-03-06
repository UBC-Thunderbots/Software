import pytest

import software.python_bindings as tbots_cpp
from software.simulated_tests.pytest_validations.robot_at_position import (
    RobotEventuallyAtPosition,
)
from software.simulated_tests.pytest_validations.ball_kicked_in_direction import (
    BallEventuallyKickedInDirection,
)
from software.simulated_tests.pytest_validations.robot_at_orientation import (
    RobotEventuallyAtOrientation,
)
from software.simulated_tests.pytest_validations.ball_is_off_ground import (
    BallIsEventuallyOffGround,
)
from software.simulated_tests.pytest_validations.robot_at_angular_velocity import (
    RobotEventuallyAtAngularVelocity,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.import_all_protos import *


def test_move_across_field(simulated_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)
    destination = tbots_cpp.Point(2.5, -1.1)
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    initial_position,
                ],
                yellow_robot_locations=[
                    tbots_cpp.Point(1, 0),
                    tbots_cpp.Point(1, 2.5),
                    tbots_cpp.Point(1, -2.5),
                    field.enemyGoalCenter(),
                    field.enemyDefenseArea().negXNegYCorner(),
                    field.enemyDefenseArea().negXPosYCorner(),
                ],
                ball_location=tbots_cpp.Point(4.5, -3),
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: MoveTactic(
                    destination=tbots_cpp.createPointProto(destination),
                    final_orientation=tbots_cpp.createAngleProto(
                        tbots_cpp.Angle.zero()
                    ),
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            # TODO: should also validate that the robot stays at destination for 1000 ticks after
            RobotEventuallyAtPosition(1, destination),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
    )


def test_autochip_move(simulated_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)
    destination = tbots_cpp.Point(0, 1.5)
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    initial_position,
                ],
                yellow_robot_locations=[
                    tbots_cpp.Point(1, 0),
                    tbots_cpp.Point(1, 2.5),
                    tbots_cpp.Point(1, -2.5),
                    field.enemyGoalCenter(),
                    field.enemyDefenseArea().negXNegYCorner(),
                    field.enemyDefenseArea().negXPosYCorner(),
                ],
                ball_location=destination,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: MoveTactic(
                    destination=tbots_cpp.createPointProto(destination),
                    final_orientation=tbots_cpp.createAngleProto(
                        tbots_cpp.Angle.zero()
                    ),
                    dribbler_mode=DribblerMode.OFF,
                    ball_collision_type=BallCollisionType.ALLOW,
                    auto_chip_or_kick=AutoChipOrKick(autochip_distance_meters=2.0),
                    max_allowed_speed_mode=MaxAllowedSpeedMode.COLLISIONS_ALLOWED,
                    obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            # TODO (#2558): should also validate that the robot stays at destination for 1000 ticks after
            RobotEventuallyAtPosition(1, destination),
            BallEventuallyKickedInDirection(tbots_cpp.Angle.zero()),
            BallIsEventuallyOffGround(),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        test_timeout_s=10,
    )


def test_autokick_move(simulated_test_runner):
    initial_position = tbots_cpp.Point(-1, -0.5)
    destination = tbots_cpp.Point(-1, -1)
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[initial_position],
                blue_robot_orientations=[tbots_cpp.Angle.threeQuarter()],
                yellow_robot_locations=[
                    tbots_cpp.Point(1, 0),
                    tbots_cpp.Point(1, 2.5),
                    tbots_cpp.Point(1, -2.5),
                    field.enemyGoalCenter(),
                    field.enemyDefenseArea().negXNegYCorner(),
                    field.enemyDefenseArea().negXPosYCorner(),
                ],
                ball_location=destination,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: MoveTactic(
                    destination=tbots_cpp.createPointProto(destination),
                    final_orientation=tbots_cpp.createAngleProto(
                        tbots_cpp.Angle.threeQuarter()
                    ),
                    dribbler_mode=DribblerMode.OFF,
                    ball_collision_type=BallCollisionType.ALLOW,
                    auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=3.0),
                    max_allowed_speed_mode=MaxAllowedSpeedMode.COLLISIONS_ALLOWED,
                    obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            # TODO (#2558): should also validate that the robot stays at destination for 1000 ticks after
            RobotEventuallyAtPosition(0, destination),
            BallEventuallyKickedInDirection(tbots_cpp.Angle.threeQuarter()),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        test_timeout_s=5,
    )


@pytest.mark.parametrize(
    "orientation, initial_position, destination, angular_velocity",
    [
        # Robot facing right, should rotate counter clockwise to face up
        (
            tbots_cpp.Angle.quarter(),
            tbots_cpp.Point(2, 0),
            tbots_cpp.Point(-2, 0),
            tbots_cpp.Angle.fromDegrees(360),
        ),
        # Robot facing right, should rotate clockwise to face down
        (
            tbots_cpp.Angle.threeQuarter(),
            tbots_cpp.Point(2, 0),
            tbots_cpp.Point(-2, 0),
            tbots_cpp.Angle.fromDegrees(-360),
        ),
        # Robot facing right, should rotate counter clockwise to face up
        (
            tbots_cpp.Angle.quarter(),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Angle.fromDegrees(360),
        ),
        # Robot facing right, should rotate clockwise to face down
        (
            tbots_cpp.Angle.threeQuarter(),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Angle.fromDegrees(-360),
        ),
        # Robot facing right, should rotate counter clockwise to face left, slightly up
        (
            tbots_cpp.Angle.fromDegrees(175),
            tbots_cpp.Point(3, -1),
            tbots_cpp.Point(0, 1),
            tbots_cpp.Angle.fromDegrees(360),
        ),
        # Robot facing right, should rotate clockwise to face down
        (
            tbots_cpp.Angle.fromDegrees(185),
            tbots_cpp.Point(3, -1),
            tbots_cpp.Point(0, 1),
            tbots_cpp.Angle.fromDegrees(-360),
        ),
    ],
)
def test_spinning_move(
    orientation, initial_position, destination, angular_velocity, simulated_test_runner
):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[initial_position],
                yellow_robot_locations=[tbots_cpp.Point(4, 0)],
                ball_location=tbots_cpp.Point(1, 1),
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: MoveTactic(
                    destination=tbots_cpp.createPointProto(destination),
                    final_orientation=tbots_cpp.createAngleProto(orientation),
                    dribbler_mode=DribblerMode.OFF,
                    ball_collision_type=BallCollisionType.ALLOW,
                    auto_chip_or_kick=AutoChipOrKick(),
                    max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                    obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            # high threshold just to check direction of angular velocity
            RobotEventuallyAtAngularVelocity(0, angular_velocity, 4),
            RobotEventuallyAtPosition(0, destination),
            RobotEventuallyAtOrientation(0, orientation),
            # TODO: validate position and orientation for 1000 ticks
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
