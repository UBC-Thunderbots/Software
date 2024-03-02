import sys

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.friendly_team_scored import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.kicker_double_touch import *
from software.simulated_tests.ball_kicked import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


def corner_kick_play_test_setup(ball_initial_pos, blue_bots, simulated_test_runner):
    """
        Setup for Corner Kick Play Test
        :param ball_initial_pos: initial position of the ball
        :param blue_bots: positions of blue robots
        :param simulated_test_runner: the simulated test runner currently being used
    """

    yellow_bots = [
        tbots_cpp.Point(1, 0),
        tbots_cpp.Point(1, 2.5),
        tbots_cpp.Point(1, -2.5),
        tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
    ]

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.NORMAL_START, team=Team.BLUE
    )
    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.DIRECT, team=Team.BLUE
    )

    # Force play override here
    blue_play = Play()
    blue_play.name = PlayName.FreeKickPlay

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
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )


def test_corner_kick_play_bottom_left(simulated_test_runner):
    simulated_test_runner.run_test(
        setup=lambda ball_and_bots_pos: corner_kick_play_test_setup(
            ball_and_bots_pos["ball_initial_pos"],
            ball_and_bots_pos["blue_bots"],
            simulated_test_runner,
        ),
        params=[
            {
                "ball_initial_pos": tbots_cpp.Point(-4.4, -2.9),
                "blue_bots": [
                    tbots_cpp.Point(-3, 2.5),
                    tbots_cpp.Point(-3, 1.5),
                    tbots_cpp.Point(-3, 0.5),
                    tbots_cpp.Point(-3, -0.5),
                    tbots_cpp.Point(-3, -1.5),
                    tbots_cpp.Point(-4.6, -3.1),
                ],
            }
        ],
        inv_always_validation_sequence_set=[
            [
                # Kicker should never double touch the ball
                KickerAlwaysNotDoubleTouch(),
            ]
        ],
        inv_eventually_validation_sequence_set=[
            [
                # Ball should eventually be kicked
                BallEventuallyKicked(),
            ]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [
                # Corner kick should result in a successful pass to a teammate
                FriendlyEventuallyHasBallPossession(),
                # Corner kick should result in a goal
                FriendlyTeamEventuallyScored(),
            ]
        ],
        test_timeout_s=10,
    )


def test_corner_kick_play_top_right(simulated_test_runner):
    simulated_test_runner.run_test(
        setup=lambda ball_and_bots_pos, runner=simulated_test_runner: corner_kick_play_test_setup(
            ball_and_bots_pos["ball_initial_pos"],
            ball_and_bots_pos["blue_bots"],
            simulated_test_runner,
        ),
        params=[
            {
                "ball_initial_pos": tbots_cpp.Point(4.4, 2.9),
                "blue_bots": [
                    tbots_cpp.Point(-3, 2.5),
                    tbots_cpp.Point(0, 1.5),
                    tbots_cpp.Point(0, 0.5),
                    tbots_cpp.Point(0, -0.5),
                    tbots_cpp.Point(0, -1.5),
                    tbots_cpp.Point(4.6, 3.1),
                ],
            }
        ],
        inv_always_validation_sequence_set=[
            [
                # Kicker should never double touch the ball
                KickerAlwaysNotDoubleTouch(),
            ]
        ],
        inv_eventually_validation_sequence_set=[
            [
                # Ball should eventually be kicked
                BallEventuallyKicked(),
            ]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [
                # Corner kick should result in a successful pass to a teammate
                FriendlyEventuallyHasBallPossession(),
                # Corner kick should result in a goal
                FriendlyTeamEventuallyScored(),
            ]
        ],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
