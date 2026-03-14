import sys

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_from_rest import *
from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.or_validation import OrValidation


def setup_kickoff_attackers():
    return [
        tbots_cpp.Point(-3, 2.5),
        tbots_cpp.Point(-3, 1.5),
        tbots_cpp.Point(-3, 0.5),
        tbots_cpp.Point(-3, -0.5),
        tbots_cpp.Point(-3, -1.5),
        tbots_cpp.Point(-3, -2.5),
    ]


def setup_kickoff_defenders():
    return [
        tbots_cpp.Point(1, 0.5),
        tbots_cpp.Point(1, 2.5),
        tbots_cpp.Point(1, -2.5),
        tbots_cpp.Point(2, -1.5),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
    ]


def init_world_state(runner, blue_bots, yellow_bots):
    runner.set_world_state(
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=tbots_cpp.Point(0, 0),
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )


@pytest.mark.parametrize("is_friendly_test", [True, False])
def test_blue_kickoff_chip(simulated_test_runner, is_friendly_test):
    ball_initial_pos = tbots_cpp.Point(0, 0)
    field = tbots_cpp.Field.createSSLDivisionBField()

    init_world_state(
        simulated_test_runner,
        setup_kickoff_attackers() if is_friendly_test else setup_kickoff_defenders(),
        setup_kickoff_defenders() if is_friendly_test else setup_kickoff_defenders(),
    )

    blue_play = Play()
    blue_play.name = PlayName.KickoffFriendlyPlay if is_friendly_test else PlayName.KickoffEnemyPlay

    yellow_play = Play()
    yellow_play.name = PlayName.KickoffEnemyPlay if is_friendly_test else PlayName.KickoffFriendlyPlay

    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.KICKOFF, team=Team.BLUE
    )
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.KICKOFF, team=Team.YELLOW
    )

    simulated_test_runner.set_play(blue_play, is_friendly=is_friendly_test)
    simulated_test_runner.set_play(yellow_play, is_friendly=not is_friendly_test)

    friendly_regions = [
        field.friendlyHalf() if is_friendly_test else field.enemyHalf(),
        field.friendlyGoal() if is_friendly_test else field.enemyGoal(),
        field.centerCircle(),
    ]

    ball_moves_at_rest_validation = BallAlwaysMovesFromRest(
        position=ball_initial_pos, threshold=0.05
    )

    always_validations_sequence_set = [
        [
            OrValidation(
                [
                    ball_moves_at_rest_validation,
                    NumberOfRobotsAlwaysStaysInRegion(
                        regions=[
                            tbots_cpp.Field.createSSLDivisionBField().centerCircle()
                        ],
                        req_robot_cnt=1,
                    ),
                    NumberOfRobotsAlwaysStaysInRegion(
                        regions=[
                            tbots_cpp.Field.createSSLDivisionBField().centerCircle()
                        ],
                        req_robot_cnt=0,
                    ),
                ]
            ),
            OrValidation(
                [
                    ball_moves_at_rest_validation,
                    NumberOfRobotsAlwaysStaysInRegion(
                        regions=friendly_regions, req_robot_cnt=6
                    ),
                ]
            ),
        ]
    ]

    eventually_validations_sequence_set = [
        [BallEventuallyExitsRegion(regions=[tbots_cpp.Circle(ball_initial_pos, 0.05)])]
    ]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validations_sequence_set,
        inv_always_validation_sequence_set=always_validations_sequence_set,
        ci_cmd_with_delay=[(4, Command.Type.NORMAL_START, Team.BLUE if is_friendly_test else Team.YELLOW)],
        test_timeout_s=10,
    )



if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
