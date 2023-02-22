import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_stops_in_region import *
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.ssl_gc_geometry_pb2 import Vector2

# TODO (#2599): Remove Duration parameter from test

# test duration global constant
TEST_DURATION = 20


@pytest.mark.parametrize(
    "run_enemy_ai", [(False,), (True,)]
)  # TODO (#2690): Robot gets stuck in corner of defense area
def test_two_ai_ball_placement(simulated_test_runner, run_enemy_ai):

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
    blue_play.name = PlayName.BallPlacementPlay

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)

    # We can parametrize running in ai_vs_ai mode
    if run_enemy_ai:
        yellow_play = Play()
        yellow_play.name = PlayName.EnemyBallPlacementPlay

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

    # Placement Always Validation
    placement_always_validation_sequence_set = [[]]

    # Placement Eventually Validation
    placement_eventually_validation_sequence_set = [
        [
            # Ball should arrive within 5cm of placement point
            BallEventuallyEntersRegion(regions=[tbots.Circle(ball_final_pos, 0.1)]),
            RobotEventuallyEntersRegion(regions=[tbots.Circle(ball_final_pos, 0.1)]),
        ]
    ]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=placement_eventually_validation_sequence_set,
        always_validation_sequence_set=placement_always_validation_sequence_set,
        test_timeout_s=TEST_DURATION,
    )

    # TODO (#2797): uncomment tests below to verify robots actually drop ball and exit region
    # send a free kick command
    # simulated_test_runner.gamecontroller.send_ci_input(
    #     gc_command=Command.Type.DIRECT_FREE_BLUE, team=Team.BLUE
    # )

    # # Drop Ball Always Validation
    # drop_ball_always_validation_sequence_set = [
    #     [
    #         BallAlwaysStaysInRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.1)]),
    #     ]
    # ]

    # # Drop Ball Eventually Validation
    # # Direct free kick after ball placement, the robot must be 0.05 away from the ball after the placement
    # # See detailed rules here: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement
    # drop_ball_eventually_validation_sequence_set = [
    #     [
    #         # Ball should arrive within 5cm of placement point
    #         BallEventuallyStopsInRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.05)]),
    #         RobotEventuallyExitsRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.05)]),
    #     ]
    # ]

    # simulated_test_runner.run_test(
    #     eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
    #     always_validation_sequence_set=drop_ball_always_validation_sequence_set,
    #     test_timeout_s=TEST_DURATION,
    # )


@pytest.mark.parametrize("run_enemy_ai", [(False,), (True,)])
def test_force_start_ball_placement(simulated_test_runner, run_enemy_ai):

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
    blue_play.name = PlayName.BallPlacementPlay

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)

    # We can parametrize running in ai_vs_ai mode
    if run_enemy_ai:
        yellow_play = Play()
        yellow_play.name = PlayName.EnemyBallPlacementPlay

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

    # Placement Always Validation
    placement_always_validation_sequence_set = [[]]

    # Placement Eventually Validation
    placement_eventually_validation_sequence_set = [
        [
            # Ball should arrive within 5cm of placement point
            BallEventuallyEntersRegion(regions=[tbots.Circle(ball_final_pos, 0.1)]),
            RobotEventuallyEntersRegion(regions=[tbots.Circle(ball_final_pos, 0.1)]),
        ]
    ]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=placement_eventually_validation_sequence_set,
        always_validation_sequence_set=placement_always_validation_sequence_set,
        test_timeout_s=TEST_DURATION,
    )

    # TODO (#2797): uncomment tests below to verify robots actually drop ball and exit region
    # send a non free kick command after the ball ball placement command
    # simulated_test_runner.gamecontroller.send_ci_input(
    #     gc_command=Command.Type.FORCE_START, team=Team.BLUE
    # )

    # # Drop Ball Always Validation
    # drop_ball_always_validation_sequence_set = [
    #     [
    #         BallAlwaysStaysInRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.1)]),
    #     ]
    # ]

    # # Drop Ball Eventually Validation
    # # Non free kick after ball placement, the robot must be 0.5 away from the ball after the placement
    # # See detailed rules here: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement
    # drop_ball_eventually_validation_sequence_set = [
    #     [
    #         # Ball should arrive within 5cm of placement point
    #         BallEventuallyStopsInRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.05)]),
    #         RobotEventuallyExitsRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.5)]),
    #     ]
    # ]

    # simulated_test_runner.run_test(
    #     eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
    #     always_validation_sequence_set=drop_ball_always_validation_sequence_set,
    #     test_timeout_s=TEST_DURATION,
    # )


if __name__ == "__main__":
    pytest_main(__file__)
