import sys

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


# TODO issue  #2599 - Remove Duration parameter from test
# @pytest.mark.parametrize("run_enemy_ai,test_duration", [(False, 20), (True, 20)])
def test_halt_play(simulated_test_runner):
    def setup(*args):
        # starting point must be Point
        ball_initial_pos = tbots_cpp.Point(0, 0)

        # Setup Bots
        blue_bots = [
            tbots_cpp.Point(-3, 2.5),
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(-3, 0.5),
            tbots_cpp.Point(-3, -0.5),
            tbots_cpp.Point(-3, -1.5),
            tbots_cpp.Point(-3, -2.5),
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
            gc_command=Command.Type.FORCE_START, team=Team.UNKNOWN
        )

        # Structure for a delayed call is tuple (delay in seconds, command, team)
        (3, Command.Type.HALT, Team.BLUE)
        (3, Command.Type.HALT, Team.YELLOW)

        # No plays to override. AI does whatever for 3 seconds before HALT CMD
        # is issued

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

    # params just have to be a list of length 1 to ensure the test runs at least once
    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[
            [RobotSpeedEventuallyBelowThreshold(1e-3)]
        ],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [RobotSpeedEventuallyBelowThreshold(1e-3)]
        ],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
