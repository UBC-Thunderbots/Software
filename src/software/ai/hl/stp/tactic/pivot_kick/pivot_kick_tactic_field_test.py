import software.python_bindings as tbots_cpp

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.ball_kicked_in_direction import (
    BallEventuallyKickedInDirection,
)


def test_pivot_kick(gameplay_test_runner):
    robot_id = 0

    ball_location = tbots_cpp.Point(-1.13, 0.75)
    angle_to_kick_at = tbots_cpp.Angle.threeQuarter()

    def setup():
        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[tbots_cpp.Point(1, 1)],
                yellow_robot_locations=[],
                ball_location=ball_location,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )
        gameplay_test_runner.set_tactics(
            blue_tactics={
                robot_id: PivotKickTactic(
                    kick_origin=tbots_cpp.createPointProto(ball_location),
                    kick_direction=tbots_cpp.createAngleProto(angle_to_kick_at),
                    auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=5.0),
                )
            }
        )

    gameplay_test_runner.run_test(
        setup=setup,
        eventually_validation_sequence_set=[
            [BallEventuallyKickedInDirection(angle_to_kick_at)]
        ],
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
