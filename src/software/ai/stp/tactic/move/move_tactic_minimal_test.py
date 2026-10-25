import proto.import_all_protos as protos
import pytest
import software.python_bindings as tbots_cpp
from proto.message_translation.tbots_protobuf import create_world_state
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.ball_is_off_ground import (
    BallIsEventuallyOffGround,
)
from software.gameplay_tests.validation.ball_kicked_in_direction import (
    BallEventuallyKickedInDirection,
)
from software.gameplay_tests.validation.duration_validation import DurationValidation
from software.gameplay_tests.validation.robot_at_angular_velocity import (
    RobotEventuallyAtAngularVelocity,
)
from software.gameplay_tests.validation.robot_at_orientation import (
    RobotEventuallyAtOrientation,
)
from software.gameplay_tests.validation.robot_at_position import (
    RobotEventuallyAtPosition,
)


def test_move_across_field(gameplay_test_runner):
    initial_position = tbots_cpp.Point(-3, 1.5)
    destination = tbots_cpp.Point(2.5, -1.1)
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup():
        gameplay_test_runner.set_world_state(
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

        gameplay_test_runner.set_tactics(
            blue_tactics={
                1: protos.MoveTactic(
                    destination=tbots_cpp.createPointProto(destination),
                    final_orientation=tbots_cpp.createAngleProto(
                        tbots_cpp.Angle.zero()
                    ),
                    dribbler_mode=protos.DribblerMode.OFF,
                    ball_collision_type=protos.BallCollisionType.AVOID,
                    auto_chip_or_kick=protos.AutoChipOrKick(),
                    max_allowed_speed_mode=protos.MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                    obstacle_avoidance_mode=protos.ObstacleAvoidanceMode.AGGRESSIVE,
                )
            }
        )

    eventually_validation_sequence_set = [
        # Robot stays at destination for one second
        [
            DurationValidation(
                duration_s=1, validation=RobotEventuallyAtPosition(1, destination)
            )
        ]
    ]

    gameplay_test_runner.run_test(
        setup=setup,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        test_timeout_s=5,
    )


if __name__ == "__main__":
    pytest_main(__file__)
