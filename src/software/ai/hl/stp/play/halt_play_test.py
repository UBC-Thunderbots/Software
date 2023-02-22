import sys

import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team

# TODO issue  #2599 - Remove Duration parameter from test
# @pytest.mark.parametrize("run_enemy_ai,test_duration", [(False, 20), (True, 20)])
def test_halt_play(simulated_test_runner):

    # starting point must be Point
    ball_initial_pos = tbots.Point(0, 0)

    # Setup Bots
    blue_bots = [
        tbots.Point(-3, 2.5),
        tbots.Point(-3, 1.5),
        tbots.Point(-3, 0.5),
        tbots.Point(-3, -0.5),
        tbots.Point(-3, -1.5),
        tbots.Point(-3, -2.5),
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
        gc_command=Command.Type.FORCE_START, team=Team.UNKNOWN
    )

    # Structure for a delayed call is tuple (delay in seconds, command, team)
    blue_delayed_ci_call = (3, Command.Type.HALT, Team.BLUE)
    yellow_delayed_ci_call = (3, Command.Type.HALT, Team.YELLOW)

    # No plays to override. AI does whatever for 3 seconds before HALT CMD
    # is issued

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
    eventually_validation_sequence_set = [[RobotSpeedEventuallyBelowThreshold(1e-3)]]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=5,
        ci_cmd_with_delay=[blue_delayed_ci_call, yellow_delayed_ci_call],
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
