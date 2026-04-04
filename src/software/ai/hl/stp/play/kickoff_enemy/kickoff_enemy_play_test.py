import threading

import software.python_bindings as tbots_cpp
from proto.play_pb2 import PlayName

from software.simulated_tests.validation.robot_enters_region import (
    NumberOfRobotsEventuallyEntersRegion,
    NumberOfRobotsAlwaysStaysInRegion,
    RobotNeverEntersRegion,
)
from software.simulated_tests.validation.or_validation import OrValidation
from software.simulated_tests.validation.ball_enters_region import BallNeverEntersRegion
from proto.message_translation.tbots_protobuf import create_world_state
from proto.import_all_protos import Command
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_kickoff_enemy_play(simulated_test_runner):
    ball_initial_pos = tbots_cpp.Point(0, 0)
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        blue_bots = [
            tbots_cpp.Point(-3, 2.5),
            tbots_cpp.Point(-2.8, 2.5),
            tbots_cpp.Point(-2.6, 2.5),
            tbots_cpp.Point(-3, -2.5),
            tbots_cpp.Point(-2.8, -2.5),
            tbots_cpp.Point(-2.6, -2.5),
        ]

        yellow_bots = [
            tbots_cpp.Point(2, 0),
            tbots_cpp.Point(2, 2.5),
            tbots_cpp.Point(2, -2.5),
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
            gc_command=Command.Type.KICKOFF, team=Team.YELLOW
        )

        # Let robots get ready before starting kickoff
        threading.Timer(
            4.0,
            lambda: simulated_test_runner.send_gamecontroller_command(
                gc_command=Command.Type.NORMAL_START, team=Team.YELLOW
            ),
        ).start()

        simulated_test_runner.set_plays(
            blue_play=PlayName.KickoffEnemyPlay,
            yellow_play=PlayName.KickoffFriendlyPlay,
        )

    # Two friendly robots in position to shadow enemy robots. Rectangles are
    # chosen to be generally in the way of the the front 3 enemy robots and the
    # friendly goal, based on where the enemy robots are initialized in the test.
    shadowing_rect_1 = tbots_cpp.Rectangle(
        tbots_cpp.Point(-0.4, 1.0), tbots_cpp.Point(0, 1.5)
    )
    shadowing_rect_2 = tbots_cpp.Rectangle(
        tbots_cpp.Point(-0.4, -1.5), tbots_cpp.Point(0, -1.0)
    )
    shadowing_rect_3 = tbots_cpp.Rectangle(
        tbots_cpp.Point(-0.86, -0.1), tbots_cpp.Point(-0.60, 0.1)
    )

    # Two Friendly robots defending the exterior of defense box
    robots_defending_rect = tbots_cpp.Rectangle(
        tbots_cpp.Point(-3.5, -1.1), tbots_cpp.Point(-3.2, 1.1)
    )

    eventually_validation_sequence_set = [
        [
            NumberOfRobotsEventuallyEntersRegion(
                regions=[shadowing_rect_1], req_robot_cnt=1
            )
        ],
        [
            NumberOfRobotsEventuallyEntersRegion(
                regions=[shadowing_rect_2], req_robot_cnt=1
            )
        ],
        [
            NumberOfRobotsEventuallyEntersRegion(
                regions=[shadowing_rect_3], req_robot_cnt=1
            )
        ],
        [
            NumberOfRobotsEventuallyEntersRegion(
                regions=[robots_defending_rect], req_robot_cnt=2
            ),
        ],
    ]

    friendly_half = field.friendlyHalf()
    center_circle = field.centerCircle()

    # Validation RoboCup SSL rules: can't enter center circle nor enemy half when ball is not kicked yet
    always_validation_sequence_set = [
        [
            OrValidation(
                [
                    RobotNeverEntersRegion(regions=[center_circle]),
                    BallNeverEntersRegion(
                        regions=[tbots_cpp.Circle(tbots_cpp.Point(0, 0), 0.05)]
                    ),
                ]
            ),
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
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
