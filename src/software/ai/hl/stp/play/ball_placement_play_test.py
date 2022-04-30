import sys

import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.ssl_gc_geometry_pb2 import Vector2

def test_ball_placement(simulated_test_runner):

    ball_initial_pos= tbots.Point(2,2) # the starting point
    # ball_final_pos = tbots.Point(-1,0) # the placement point
    
    blue_bots = [
        tbots.Point(-2,1.8),
        tbots.Point(-2,1.2),
        tbots.Point(-2,0.6),
        tbots.Point(-2,-0.6),
        tbots.Point(-2,-1.2),
        tbots.Point(-2,-1.8),
    ]

    yellow_bots = [

    ]

    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.BALL_PLACEMENT, 
        team=Team.BLUE,
        ball_placement_pos_vec=Vector2(x=-1,y=0)
    )

    # game_state = GameState()
    # command = Command()
    # state = State()

    

    # game_state.command =  # ball placement us
    # game_state.ball_placement_point.x_meters = -1
    # game_state.ball_placement_point.y_meters = 0
    

    # Setup play
    play = Play()
    play.name = Play.BallPlacementPlay
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        Play, play
    )
    # Setup ball with initial velocity using our software/geom
    
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=[],
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=tbots.Vector(0,0))
    )

    # Always Validation
    always_validation_sequence_set = [
        [
            BallNeverEntersRegion(regions=[tbots.Field.createSSLDivisionBField().friendlyGoal()]),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            BallEventuallyEntersRegion(regions=[tbots.Field.createSSLDivisionBField().friendlyHalf()]),
        ]
    ]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-svv"]))