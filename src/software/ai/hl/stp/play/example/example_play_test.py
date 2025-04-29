import sys

import pytest

import software.python_bindings as tbots_cpp
from software.simulated_tests.robot_enters_region import NumberOfRobotsEventuallyExitsRegion, NumberOfRobotsEventuallyEntersRegion
from software.simulated_tests.robot_speed_threshold import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.play_pb2 import Play, PlayName

# TODO issue  #2599 - Remove Duration parameter from test
# @pytest.mark.parametrize("run_enemy_ai,test_duration", [(False, 20), (True, 20)])
def test_example_play(simulated_test_runner):
    ball_initial_pos = tbots_cpp.Point(0,0)
    def setup(*args):
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
            tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXNegYCorner(),
            tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXPosYCorner(),
        ]

        # Force play override here
        blue_play = Play()
        blue_play.name = PlayName.ExamplePlay

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            Play, yellow_play
        )

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
        inv_eventually_validation_sequence_set=[[NumberOfRobotsEventuallyEntersRegion(
            region=tbots_cpp.Circle(ball_initial_pos, 1.1),
            req_robot_cnt=6
        ), NumberOfRobotsEventuallyExitsRegion(
            region=tbots_cpp.Circle(ball_initial_pos, 0.9),
            req_robot_cnt=6
        )]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))