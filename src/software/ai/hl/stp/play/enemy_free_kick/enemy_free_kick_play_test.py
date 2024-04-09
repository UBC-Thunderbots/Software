import sys

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName

from software.simulated_tests.ball_moves_from_rest import (
    BallNeverMovesFromRest,
    BallEventuallyMovesFromRest,
)
from software.simulated_tests.friendly_team_scored import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.robot_enters_region import (
    RobotEventuallyEntersRegion,
    RobotNeverEntersRegion,
)
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "blue_bots,yellow_bots,ball_initial_pos",
    [
        (
            [
                tbots_cpp.Point(-2.75, 2.5),
                tbots_cpp.Point(-2.75, 1.5),
                tbots_cpp.Point(-2.75, 0.5),
                tbots_cpp.Point(-2.75, -0.5),
                tbots_cpp.Point(-2.75, -1.5),
                tbots_cpp.Point(-2.75, -3.0),
            ],
            [
                tbots_cpp.Point(1, 0),
                tbots_cpp.Point(1, 2.5),
                tbots_cpp.Point(1, -2.5),
                tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXNegYCorner(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXPosYCorner(),
            ],
            tbots_cpp.Point(0, 0),
        ),
        (
            [
                tbots_cpp.Point(-2.75, 2.5),
                tbots_cpp.Point(-2.75, 1.5),
                tbots_cpp.Point(-2.75, 0.5),
                tbots_cpp.Point(-2.75, -0.5),
                tbots_cpp.Point(-2.75, -1.5),
                tbots_cpp.Point(-2.75, -3.0),
            ],
            [
                tbots_cpp.Point(-1.8, 2),
                tbots_cpp.Point(0, 2.5),
                tbots_cpp.Point(-2, 0),
                tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXNegYCorner(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXPosYCorner(),
            ],
            tbots_cpp.Point(-2, 2),
        ),
        (
            [
                tbots_cpp.Point(-2.75, 2.5),
                tbots_cpp.Point(-2.75, 1.5),
                tbots_cpp.Point(-2.75, 0.5),
                tbots_cpp.Point(-2.75, -0.5),
                tbots_cpp.Point(-2.75, -1.5),
                tbots_cpp.Point(-2.75, -3.0),
            ],
            [
                tbots_cpp.Point(1.8, -2),
                tbots_cpp.Point(0, -2.5),
                tbots_cpp.Point(2, 0),
                tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXNegYCorner(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXPosYCorner(),
            ],
            tbots_cpp.Point(2, -2),
        ),
    ],
)
def test_enemy_free_kick_play(
    simulated_test_runner, blue_bots, yellow_bots, ball_initial_pos
):
    # starting point must be Point
    # placement point must be Vector2 to work with game controller

    # Setup Bots
    def setup(*args):

        # Game Controller Setup
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.DIRECT, team=Team.YELLOW
        )

        # Force play override here
        blue_play = Play()
        blue_play.name = PlayName.EnemyFreeKickPlay

        yellow_play = Play()
        yellow_play.name = PlayName.FreeKickPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            Play, yellow_play
        )

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

    # Always Validation
    always_validation_sequence_set = [
        [#RobotNeverEntersRegion(regions=[tbots_cpp.Circle(ball_initial_pos, 0.5)]),
            BallNeverMovesFromRest(position=ball_initial_pos)]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [#RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(ball_initial_pos, 1)]),
         ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        params=[0, 1, 2],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=5,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
