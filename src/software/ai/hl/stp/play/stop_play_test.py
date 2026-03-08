import pytest

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from proto.play_pb2 import Play, PlayName
from proto.ssl_gc_common_pb2 import Team
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.robot_speed_threshold import (
    RobotSpeedAlwaysBelowThreshold,
)
from software.simulated_tests.pytest_validations.robot_enters_region import (
    RobotNeverEntersRegion,
)
from software.simulated_tests.pytest_validations.delay_validation import DelayValidation
from software.simulated_tests.simulated_test_fixture import pytest_main


@pytest.mark.parametrize(
    "ball_position, blue_robot_positions",
    [
        # test_stop_play_ball_at_centre_robots_spread_out
        (
            tbots_cpp.Point(0, 0),
            [
                tbots_cpp.Point(-4, 0),
                tbots_cpp.Point(-0.3, 0),
                tbots_cpp.Point(0.3, 0),
                tbots_cpp.Point(0, 0.3),
                tbots_cpp.Point(-3, -1.5),
                tbots_cpp.Point(4.6, -3.1),
            ],
        ),
        # test_stop_play_friendly_half_robots_spread_out
        (
            tbots_cpp.Point(-1, 0),
            [
                tbots_cpp.Point(-4, 0),
                tbots_cpp.Point(-2.3, 0),
                tbots_cpp.Point(-1.7, 0),
                tbots_cpp.Point(-2, 0.3),
                tbots_cpp.Point(-3, -1.5),
                tbots_cpp.Point(4.6, -3.1),
            ],
        ),
        # test_stop_play_friendly_half_corner_robots_close_together
        (
            tbots_cpp.Point(-4, -2.5),
            [
                tbots_cpp.Point(-3, -2.5),
                tbots_cpp.Point(-4, -2),
                tbots_cpp.Point(-2, -2.5),
                tbots_cpp.Point(-3, -2),
                tbots_cpp.Point(-3.5, -2),
                tbots_cpp.Point(-3, -1),
            ],
        ),
        # test_stop_play_enemy_half_robots_spread_out
        (
            tbots_cpp.Point(2, 0),
            [
                tbots_cpp.Point(-4, 0),
                tbots_cpp.Point(1.7, 0),
                tbots_cpp.Point(2.3, 0),
                tbots_cpp.Point(2, 0.3),
                tbots_cpp.Point(-3, -1.5),
                tbots_cpp.Point(3, -3),
            ],
        ),
        # test_stop_play_enemy_half_corner_robots_close_together
        (
            tbots_cpp.Point(4, -2.5),
            [
                tbots_cpp.Point(2, -2.5),
                tbots_cpp.Point(4, -1),
                tbots_cpp.Point(3, -2.5),
                tbots_cpp.Point(3, -2),
                tbots_cpp.Point(3.5, -2),
                tbots_cpp.Point(3, -1),
            ],
        ),
        # test_stop_play_centre_robots_close_together
        (
            tbots_cpp.Point(0, 0),
            [
                tbots_cpp.Point(-2, 0),
                tbots_cpp.Point(0, 0.3),
                tbots_cpp.Point(0.3, 0),
                tbots_cpp.Point(0, -0.3),
                tbots_cpp.Point(-0.3, 0),
                tbots_cpp.Point(0.2, 0.2),
            ],
        ),
        # test_stop_play_ball_in_front_of_enemy_defense_area
        (
            tbots_cpp.Point(3, 0),
            [
                tbots_cpp.Point(-4.5, 2),
                tbots_cpp.Point(0, 0.3),
                tbots_cpp.Point(0.3, 0),
                tbots_cpp.Point(0, -0.3),
                tbots_cpp.Point(-0.3, 0),
                tbots_cpp.Point(0.2, 0.2),
            ],
        ),
        # test_stop_play_ball_in_front_of_friendly_defense_area
        # TODO (#3630): robots stop within 0.5m of ball
        # (
        #     tbots_cpp.Point(-3, 0),
        #     [
        #         tbots_cpp.Point(-4.5, 2),
        #         tbots_cpp.Point(0, 3),
        #         tbots_cpp.Point(-1, -1),
        #         tbots_cpp.Point(0, 0),
        #         tbots_cpp.Point(-1, 0),
        #         tbots_cpp.Point(2, 2),
        #     ],
        # ),
    ],
)
def test_stop_play(ball_position, blue_robot_positions, simulated_test_runner):
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=blue_robot_positions,
                yellow_robot_locations=[
                    tbots_cpp.Point(1, 0),
                    tbots_cpp.Point(1, 2.5),
                    tbots_cpp.Point(1, -2.5),
                    field.enemyGoalCenter(),
                    field.enemyDefenseArea().negXNegYCorner(),
                    field.enemyDefenseArea().negXPosYCorner(),
                ],
                ball_location=ball_position,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )

        blue_play = Play()
        blue_play.name = PlayName.StopPlay
        simulated_test_runner.set_play(blue_play, is_friendly=True)

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay
        simulated_test_runner.set_play(yellow_play, is_friendly=False)

    # Wait 3 seconds for robots that start too close to the ball to move away
    # Then validate robots slow down below 1.5 m/s
    # And validate robots avoid ball (stay at least 0.5m away)
    always_stop_play_rules = [
        [
            DelayValidation(
                delay_s=3,
                validation=RobotSpeedAlwaysBelowThreshold(speed_threshold=1.5),
            ),
            DelayValidation(
                delay_s=3,
                validation=RobotNeverEntersRegion(
                    regions=[tbots_cpp.Circle(ball_position, 0.5)]
                ),
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_always_validation_sequence_set=always_stop_play_rules,
        ag_always_validation_sequence_set=always_stop_play_rules,
        test_timeout_s=6,
    )


if __name__ == "__main__":
    pytest_main(__file__)
