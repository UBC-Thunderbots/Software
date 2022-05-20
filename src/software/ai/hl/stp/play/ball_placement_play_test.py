import sys

import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.ssl_gc_geometry_pb2 import Vector2

# TODO issue  #2599 - Remove Duration parameter from test
@pytest.mark.parametrize("run_enemy_ai,test_duration", [(False, 20), (True, 20)])
def test_two_ai_ball_placement(simulated_test_runner, run_enemy_ai, test_duration):

    # starting point must be Point
    ball_initial_pos = tbots.Point(2, 2)
    # placement point must be Vector2 to work with game controller
    ball_final_pos = tbots.Point(-3, -2)

    # Setup Bots
    blue_bots = [
        tbots.Point(-2.75, 2.5),
        tbots.Point(-2.75, 1.5),
        tbots.Point(-2.75, 0.5),
        tbots.Point(-2.75, -0.5),
        tbots.Point(-2.75, -1.5),
        tbots.Point(4.6, -3.1),
    ]

    yellow_bots = [
        tbots.Point(1, 0),
        tbots.Point(1, 2.5),
        tbots.Point(1, -2.5),
        tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
        tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
        tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
    ]

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )
    # Pass in placement point here - not required for all play tests
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.BALL_PLACEMENT,
        team=Team.BLUE,
        final_ball_placement_point=ball_final_pos,
    )

    # Force play override here
    blue_play = Play()
    blue_play.name = Play.BallPlacementPlay

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)

    # We can parametrize running in ai_vs_ai mode
    if run_enemy_ai:
        yellow_play = Play()
        yellow_play.name = Play.EnemyBallPlacementPlay

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
            ball_velocity=tbots.Vector(0, 0),
        ),
    )

    # Always Validation
    always_validation_sequence_set = [[]]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            # Ball should arrive within 5cm of placement point
            BallEventuallyEntersRegion(regions=[tbots.Circle(ball_final_pos, 0.05)]),
        ]
    ]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=test_duration,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
