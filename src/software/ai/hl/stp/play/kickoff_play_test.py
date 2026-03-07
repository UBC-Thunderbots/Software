import sys

import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_from_rest import *
from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.or_validation import OrValidation


def setup_kickoff_attackers():
    return [
        tbots_cpp.Point(-3, 2.5),
        tbots_cpp.Point(-3, 1.5),
        tbots_cpp.Point(-3, 0.5),
        tbots_cpp.Point(-3, -0.5),
        tbots_cpp.Point(-3, -1.5),
        tbots_cpp.Point(-3, -2.5),
    ]


def setup_kickoff_defenders():
    return [
        tbots_cpp.Point(1, 0.5),
        tbots_cpp.Point(1, 2.5),
        tbots_cpp.Point(1, -2.5),
        tbots_cpp.Point(2, -1.5),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
    ]


def init_world_state(runner, blue_bots, yellow_bots):
    runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=tbots_cpp.Point(0, 0),
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )




def test_blue_kickoff_chip(simulated_test_runner):
    ball_initial_pos = tbots_cpp.Point(0, 0)
    field = tbots_cpp.Field.createSSLDivisionBField()

    init_world_state(
        simulated_test_runner,
        setup_kickoff_attackers(),
        setup_kickoff_defenders(),
    )

    blue_play = Play()
    blue_play.name = PlayName.KickoffFriendlyPlay

    yellow_play = Play()
    yellow_play.name = PlayName.KickoffEnemyPlay

    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.KICKOFF, team=Team.BLUE
    )
    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.KICKOFF, team=Team.YELLOW
    )

    blue_regions = [
        tbots_cpp.Field.createSSLDivisionBField().friendlyHalf(),
        tbots_cpp.Field.createSSLDivisionBField().friendlyGoal(),
        tbots_cpp.Field.createSSLDivisionBField().centerCircle()
    ]

    ball_moves_at_rest_validation = BallAlwaysMovesFromRest(
        position=ball_initial_pos, threshold=0.05
    )

    always_validations = [[
        OrValidation([
            ball_moves_at_rest_validation,
            NumberOfRobotsAlwaysStaysInRegion(
                regions=[tbots_cpp.Field.createSSLDivisionBField().centerCircle()],
                req_robot_cnt=1
            ),
            NumberOfRobotsAlwaysStaysInRegion(
                regions=[tbots_cpp.Field.createSSLDivisionBField().centerCircle()],
                req_robot_cnt=0
            ),
        ]),
        OrValidation([
            ball_moves_at_rest_validation,
            NumberOfRobotsAlwaysStaysInRegion(regions=blue_regions, req_robot_cnt=6),
        ])
    ]]

    eventually_validations = [[
        BallEventuallyExitsRegion(regions=[tbots_cpp.Circle(ball_initial_pos, 0.05)])
    ]]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validations,
        inv_always_validation_sequence_set=always_validations,
        ci_cmd_with_delay=[(4, Command.Type.NORMAL_START, Team.BLUE)],
        test_timeout_s=10,
    )



if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
