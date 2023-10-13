import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.ssl_gc_geometry_pb2 import Vector2

# TODO (#2599): Remove Duration parameter from test


def test_two_ai_ball_placement(simulated_test_runner):

    # placement point must be Vector2 to work with game controller
    ball_final_pos = tbots_cpp.Point(-3, -2)

    def setup(run_enemy_ai):
        # starting point must be Point
        ball_initial_pos = tbots_cpp.Point(2, 2)

        # Setup Bots
        blue_bots = [
            tbots_cpp.Point(-2.75, 2.5),
            tbots_cpp.Point(-2.75, 1.5),
            tbots_cpp.Point(-2.75, 0.5),
            tbots_cpp.Point(-2.75, -0.5),
            tbots_cpp.Point(-2.75, -1.5),
            tbots_cpp.Point(4.5, -3.0),
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
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

    simulated_test_runner.run_test(
        setup=setup,
        params=[False],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[
            [
                BallEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
                RobotEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
            ]
        ],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [
                BallEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
                RobotEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
            ]
        ],
        test_timeout_s=[20],
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
#             regions=[tbots.Circle(ball_final_pos, 0.15)]),
#     ]
# ]

# # Drop Ball Eventually Validation
# # Direct free kick after ball placement, the robot must be 0.05 away from the ball after the placement
# # See detailed rules here: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement
# drop_ball_eventually_validation_sequence_set = [
#     [
#         # Ball should arrive within 0.15m of placement point
#         BallEventuallyStopsInRegion(
#             regions=[tbots.Circle(ball_final_pos, 0.15)]),
#           #Robot should exit from ball at least 0.05m
#         RobotEventuallyExitsRegion(
#             regions=[tbots.Circle(ball_final_pos, 0.2)]),
#     ]
# ]

# simulated_test_runner.run_test(
#     eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
#     always_validation_sequence_set=drop_ball_always_validation_sequence_set,
#     test_timeout_s=TEST_DURATION,
# )


def test_force_start_ai_ball_placement(simulated_test_runner):

    # placement point must be Vector2 to work with game controller
    ball_final_pos = tbots_cpp.Point(-3, -2)

    def setup(run_enemy_ai):
        # starting point must be Point
        ball_initial_pos = tbots_cpp.Point(2, 2)

        # Setup Bots
        blue_bots = [
            tbots_cpp.Point(-2.75, 2.5),
            tbots_cpp.Point(-2.75, 1.5),
            tbots_cpp.Point(-2.75, 0.5),
            tbots_cpp.Point(-2.75, -0.5),
            tbots_cpp.Point(-2.75, -1.5),
            tbots_cpp.Point(4.5, -3.0),
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
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

    simulated_test_runner.run_test(
        setup=setup,
        params=[False],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[
            [
                BallEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
                RobotEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
            ]
        ],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [
                BallEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
                RobotEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_final_pos, 0.15)]
                ),
            ]
        ],
        test_timeout_s=[20],
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
    #         # Ball should arrive within 0.15m of placement point
    #         BallEventuallyStopsInRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.15)]),
    #           # Robot has to be 0.5 m away from the ball
    #         RobotEventuallyExitsRegion(
    #             regions=[tbots.Circle(ball_final_pos, 0.65)]),
    #     ]
    # ]

    # simulated_test_runner.run_test(
    #     eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
    #     always_validation_sequence_set=drop_ball_always_validation_sequence_set,
    #     test_timeout_s=TEST_DURATION,
    # )


if __name__ == "__main__":
    pytest_main(__file__)
