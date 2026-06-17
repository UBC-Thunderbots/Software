import math

import pytest
import software.python_bindings as tbots_cpp

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.delay_validation import DelayValidation
from software.gameplay_tests.validation.duration_validation import DurationValidation
from software.gameplay_tests.validation.robot_at_orientation import (
    RobotEventuallyAtOrientation,
)
from software.gameplay_tests.validation.robot_enters_region import (
    RobotEventuallyEntersRegion,
)


@pytest.mark.parametrize(
    "angle",
    [0, 45, 90, 180, 270, 0],
)
def test_basic_rotation(angle, gameplay_test_runner):
    target_angle = tbots_cpp.Angle.fromDegrees(angle)
    start_position = tbots_cpp.Point(-1.5, 0.6)

    def setup():
        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    start_position,
                ],
                yellow_robot_locations=[],
                ball_location=tbots_cpp.Point(0, 0),
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        move_tactic = MoveTactic(
            destination=tbots_cpp.createPointProto(start_position),
            dribbler_mode=DribblerMode.OFF,
            final_orientation=tbots_cpp.createAngleProto(target_angle),
            ball_collision_type=BallCollisionType.AVOID,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )

        gameplay_test_runner.set_tactics(
            blue_tactics={
                0: move_tactic,
            },
        )

    gameplay_test_runner.run_test(
        setup=setup,
        test_timeout_s=5,
        eventually_validation_sequence_set=[
            [
                DurationValidation(
                    duration_s=1,
                    validation=RobotEventuallyAtOrientation(0, target_angle),
                ),
            ]
        ],
    )


@pytest.mark.parametrize(
    "start_position, end_position",
    [
        (
            tbots_cpp.Point(-1.5, 0.6),
            tbots_cpp.Point(-0.3, 0.6),
        ),
        (
            tbots_cpp.Point(-0.3, 0.6),
            tbots_cpp.Point(-0.3, -0.6),
        ),
        (
            tbots_cpp.Point(-0.3, -0.6),
            tbots_cpp.Point(-1.5, -0.6),
        ),
        (
            tbots_cpp.Point(-1.5, -0.6),
            tbots_cpp.Point(-1.5, 0.6),
        ),
    ],
)
def test_one_robots_square(start_position, end_position, gameplay_test_runner):
    def setup():
        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    start_position,
                ],
                yellow_robot_locations=[],
                ball_location=tbots_cpp.Point(0, 0),
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        tactic = MoveTactic(
            destination=tbots_cpp.createPointProto(end_position),
            dribbler_mode=DribblerMode.OFF,
            final_orientation=Angle(radians=-math.pi / 2),
            ball_collision_type=BallCollisionType.AVOID,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
        )

        gameplay_test_runner.set_tactics(
            blue_tactics={
                0: tactic,
            },
        )

    gameplay_test_runner.run_test(
        setup=setup,
        test_timeout_s=4,
        eventually_validation_sequence_set=[
            [
                RobotEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(end_position, 0.05)]
                ),
                DelayValidation(
                    delay_s=1,
                    validation=RobotEventuallyEntersRegion(
                        regions=[tbots_cpp.Circle(end_position, 0.05)]
                    ),
                ),
            ]
        ],
    )


if __name__ == "__main__":
    pytest_main(__file__)
