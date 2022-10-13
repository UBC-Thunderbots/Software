import sys

import pytest


import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.py_constants import BALL_MAX_RADIUS_METERS
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.friendly_goal_scored import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team

def test_kickoff_play_ready(simulated_test_runner):

    # starting point must be Point
    ball_initial_pos = tbots.Field.createSSLDivisionBField().friendlyPenaltyMark()

    # Setup Bots
    blue_bots = [
        tbots.Point(-2,-2),
        tbots.Point(-3,-1),
        tbots.Point(-3, 0),
        tbots.Point(-3, 1),
        tbots.Point(-3, 2),
        tbots.Point(2, 2.5),
    ]

    yellow_bots = [
        tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
    ]

    blue_play = Play()
    yellow_play = Play()

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.PENALTY, team=Team.BLUE
    )
    # simulated_test_runner.gamecontroller.send_ci_input(
    #     gc_command=Command.Type.NORMAL_START, team=Team.BLUE
    # )

    # delayed_ci_call = (4, Command.Type.NORMAL_START, Team.BLUE)
    blue_play.name = PlayName.PenaltyKickPlay
    yellow_play.name = PlayName.HaltPlay

    # Force play override here
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

    # Always Validation
    # TODO- #2753 Validation
    always_validation_sequence_set = [[]]

    # Eventually Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=15,
        # ci_cmd_with_delay=[delayed_ci_call]
    )

def test_kickoff_play_kicking(simulated_test_runner):

    # starting point must be Point
    ball_initial_pos = tbots.Field.createSSLDivisionBField().friendlyPenaltyMark()

    # Setup Bots
    blue_bots = [
        tbots.Point(-2,-2),
        tbots.Point(-3,-1),
        tbots.Point(-3, 0),
        tbots.Point(-3, 1),
        tbots.Point(-3, 2),
        tbots.Point(2, 2.5),
    ]

    yellow_bots = [
        tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
    ]

    blue_play = Play()
    yellow_play = Play()

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.PENALTY, team=Team.BLUE
    )

    delayed_ci_call = (4, Command.Type.NORMAL_START, Team.BLUE)
    blue_play.name = PlayName.PenaltyKickPlay
    yellow_play.name = PlayName.HaltPlay

    # Force play override here
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

    # Always Validation
    # TODO- #2753 Validation
    always_validation_sequence_set = [[]]

    # Eventually Validation
    # TODO- #2753 Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=15,
        ci_cmd_with_delay=[delayed_ci_call]
    )

if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))