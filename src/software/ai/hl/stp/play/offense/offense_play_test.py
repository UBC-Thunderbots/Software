import sys

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.friendly_team_scored import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


def test_offense_play(simulated_test_runner):

    # starting point must be Point
    ball_initial_pos = tbots_cpp.Point(-4.4, 2.9)
    # placement point must be Vector2 to work with game controller
    tbots_cpp.Point(-3, -2)

    # Setup Bots
    blue_bots = [
        tbots_cpp.Point(-4.5, 3.0),
        tbots_cpp.Point(-2, 1.5),
        tbots_cpp.Point(-2, 0.5),
        tbots_cpp.Point(-2, -1.7),
        tbots_cpp.Point(-2, -1.5),
        tbots_cpp.Point(-2, -0.5),
    ]

    yellow_bots = [
        tbots_cpp.Point(1, 0),
        tbots_cpp.Point(1, 2.5),
        tbots_cpp.Point(1, -2.5),
        tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
    ]

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    # Force play override here
    blue_play = Play()
    blue_play.name = PlayName.OffensePlay

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

    # Always Validation
    # TODO- #2779 Validation
    always_validation_sequence_set = [[]]

    # Eventually Validation
    # TODO- #2779 Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=15,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
