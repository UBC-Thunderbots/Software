import sys

import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.friendly_team_scored import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


def free_kick_play_test_setup(test_setup_arg, simulated_test_runner):
    """
    Sets up the free kick play test
    :param test_setup_arg: object
        {
            blue_bots: list of positions for blue robots
            yellow_bots: list of positions for yellow robots
            ball_initial_pos: initial ball position
            play_name: name of the current play
        }
    :param: simulated_test_runner: the current test runner
    """

    blue_bots = test_setup_arg["blue_bots"]
    yellow_bots = test_setup_arg["yellow_bots"]
    ball_initial_pos = test_setup_arg["ball_initial_pos"]
    play_name = test_setup_arg["play_name"]

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.NORMAL_START, team=Team.BLUE
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.DIRECT, team=Team.BLUE
    )

    # Force play override here
    blue_play = Play()
    blue_play.name = play_name

    yellow_play = Play()
    yellow_play.name = PlayName.HaltPlay

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(Play, yellow_play)

    # Create world state
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=tbots.Vector(0, 0),
        ),
    )


# We want to test friendly half, enemy half, and at the border of the field
@pytest.mark.parametrize(
    "ball_initial_pos",
    [tbots.Point(1.5, -2.75), tbots.Point(-1.5, -2.75), tbots.Point(1.5, -3)],
)
def test_free_kick_play_friendly(simulated_test_runner, ball_initial_pos):
    # TODO- #2753 Validation
    # params just have to be a list of length 1 to ensure the test runs at least once
    simulated_test_runner.run_test(
        setup=lambda test_setup_arg: free_kick_play_test_setup(
            test_setup_arg, simulated_test_runner
        ),
        params=[
            {
                "blue_bots": [
                    tbots.Point(-4.5, 0),
                    tbots.Point(-3, 1.5),
                    tbots.Point(-3, 0.5),
                    tbots.Point(-3, -0.5),
                    tbots.Point(-3, -1.5),
                    tbots.Point(4, -2.5),
                ],
                "yellow_bots": [
                    tbots.Point(1, 0),
                    tbots.Point(1, 2.5),
                    tbots.Point(1, -2.5),
                    tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
                    tbots.Field.createSSLDivisionBField()
                    .enemyDefenseArea()
                    .negXNegYCorner(),
                    tbots.Field.createSSLDivisionBField()
                    .enemyDefenseArea()
                    .negXPosYCorner(),
                ],
                "ball_initial_pos": ball_initial_pos,
                "play_name": PlayName.FreeKickPlay,
            }
        ],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


@pytest.mark.parametrize(
    "ball_initial_pos,yellow_bots",
    [
        # not close to our net
        (
            tbots.Point(0.9, 2.85),
            [
                tbots.Point(1, 3),
                tbots.Point(-2, -1.25),
                tbots.Point(-1, -0.25),
                tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
                tbots.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXNegYCorner(),
                tbots.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXPosYCorner(),
            ],
        ),
        # close to our net
        (
            tbots.Point(-2.4, 1),
            [
                tbots.Point(-2.3, 1.05),
                tbots.Point(-3.5, 2),
                tbots.Point(-1.2, 0),
                tbots.Point(-2.3, -1),
                tbots.Point(-3.8, -2),
                tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
            ],
        ),
    ],
)
def test_free_kick_play_enemy(simulated_test_runner, ball_initial_pos, yellow_bots):

    blue_bots = [
        tbots.Point(-4.5, 0),
        tbots.Point(-3, 1.5),
        tbots.Point(-3, 0.5),
        tbots.Point(-3, -0.5),
        tbots.Point(-3, -1.5),
        tbots.Point(4, -2.5),
    ]
    # TODO- #2753 Validation
    simulated_test_runner.run_test(
        setup=lambda test_setup_arg: free_kick_play_test_setup(
            test_setup_arg, simulated_test_runner
        ),
        params=[
            {
                "blue_bots": [
                    tbots.Point(-4.5, 0),
                    tbots.Point(-3, 1.5),
                    tbots.Point(-3, 0.5),
                    tbots.Point(-3, -0.5),
                    tbots.Point(-3, -1.5),
                    tbots.Point(4, -2.5),
                ],
                "yellow_bots": yellow_bots,
                "ball_initial_pos": ball_initial_pos,
                "play_name": PlayName.EnemyFreekickPlay,
            }
        ],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


@pytest.mark.parametrize(
    "ball_initial_pos",
    [tbots.Point(1.5, -2.75), tbots.Point(-1.5, -2.75), tbots.Point(1.5, -3)],
)
def test_free_kick_play_both(simulated_test_runner, ball_initial_pos):
    # TODO- #2753 Validation
    simulated_test_runner.run_test(
        setup=lambda test_setup_arg: free_kick_play_test_setup(
            test_setup_arg, simulated_test_runner
        ),
        params=[
            {
                "blue_bots": [
                    tbots.Point(-3, 0.25),
                    tbots.Point(-3, 1.5),
                    tbots.Point(-3, 0.5),
                    tbots.Point(-3, -0.5),
                    tbots.Point(-3, -1.5),
                    tbots.Point(-3, -0.25),
                ],
                "yellow_bots": [
                    tbots.Point(3, 0.25),
                    tbots.Point(3, 1.5),
                    tbots.Point(3, 0.5),
                    tbots.Point(3, -0.5),
                    tbots.Point(3, -1.5),
                    tbots.Point(3, -0.25),
                ],
                "ball_initial_pos": ball_initial_pos,
                "play_name": PlayName.EnemyFreekickPlay,
            }
        ],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=15,
    )


free_kick_play_test_setup.__test__ = False


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
