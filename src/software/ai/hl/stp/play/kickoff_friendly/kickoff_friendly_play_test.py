import threading

import software.python_bindings as tbots_cpp
from proto.play_pb2 import PlayName

from software.simulated_tests.validation.robot_enters_region import (
    NumberOfRobotsEventuallyEntersRegion,
    NumberOfRobotsAlwaysStaysInRegion,
    NumberOfRobotsNeverEntersRegion,
    RobotEventuallyEntersRegion,
)
from software.simulated_tests.validation.ball_kicked_in_direction import (
    BallEventuallyKickedInDirection,
)
from software.simulated_tests.validation.ball_enters_region import BallNeverEntersRegion
from software.simulated_tests.validation.or_validation import OrValidation
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import Command
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_kickoff_friendly_play(simulated_test_runner):
    ball_initial_pos = tbots_cpp.Point(0, 0)

    def setup(*args):
        field = tbots_cpp.Field.createSSLDivisionBField()

        blue_bots = [
            tbots_cpp.Point(-3, 2.5),
            tbots_cpp.Point(-2.8, 2.5),
            tbots_cpp.Point(-2.6, 2.5),
            tbots_cpp.Point(-3, -2.5),
            tbots_cpp.Point(-2.8, -2.5),
            tbots_cpp.Point(-2.6, -2.5),
        ]

        yellow_bots = [
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(1, 2.5),
            tbots_cpp.Point(1, -2.5),
            field.enemyGoalCenter(),
            field.enemyDefenseArea().negXNegYCorner(),
            field.enemyDefenseArea().negXPosYCorner(),
        ]

        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=blue_bots,
                yellow_robot_locations=yellow_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.KICKOFF, team=Team.BLUE
        )

        simulated_test_runner.set_plays(
            blue_play=PlayName.KickoffFriendlyPlay,
            yellow_play=PlayName.KickoffEnemyPlay,
        )

    field = tbots_cpp.Field.createSSLDivisionBField()
    center_circle = field.centerCircle()
    friendly_half = field.friendlyHalf()

    eventually_validation_sequence_set = [
        [
            # Check robot enters center circle, then kicks ball towards enemy half
            RobotEventuallyEntersRegion(regions=[center_circle]),
            BallEventuallyKickedInDirection(
                tbots_cpp.Angle.zero(), max_angle_difference_degrees=90
            ),
        ],
        [
            # Two friendly robots near the half line setting up for offense
            NumberOfRobotsEventuallyEntersRegion(
                regions=[
                    tbots_cpp.Rectangle(
                        tbots_cpp.Point(-1.5, -2.5), tbots_cpp.Point(-0.5, 2.5)
                    )
                ],
                req_robot_cnt=2,
            ),
        ],
        [
            # Two Friendly robots defending the exterior of defense box
            NumberOfRobotsEventuallyEntersRegion(
                regions=[
                    tbots_cpp.Rectangle(
                        tbots_cpp.Point(-3.2, 1.1), tbots_cpp.Point(-3.51, -1.1)
                    )
                ],
                req_robot_cnt=2,
            ),
        ],
    ]

    # Validation RoboCup SSL rules: can't enter enemy half when ball is not kicked yet
    # Only one robot allowed to enter center circle to kick
    always_validation_sequence_set = [
        [
            OrValidation(
                [
                    NumberOfRobotsAlwaysStaysInRegion(
                        regions=[friendly_half], req_robot_cnt=6
                    ),
                    BallNeverEntersRegion(
                        regions=[tbots_cpp.Circle(tbots_cpp.Point(0, 0), 0.05)]
                    ),
                ]
            ),
            OrValidation(
                [
                    NumberOfRobotsNeverEntersRegion(
                        regions=[center_circle], req_robot_cnt=2
                    ),
                    BallNeverEntersRegion(
                        regions=[tbots_cpp.Circle(tbots_cpp.Point(0, 0), 0.05)]
                    ),
                ]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
        ci_cmd_with_delay=[(4.0,Command.Type.NORMAL_START,Team.BLUE)], # Let robots get ready before starting kickoff
    )


if __name__ == "__main__":
    pytest_main(__file__)
