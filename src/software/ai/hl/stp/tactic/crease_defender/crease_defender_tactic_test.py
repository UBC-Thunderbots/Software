import pytest

import software.python_bindings as tbots_cpp
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_in_direction import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_speed_threshold import *
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "blue_bots, "
    "enemy_threat_position, yellow_bots,"
    "ball_initial_position, ball_initial_velocity",
    [
        (
            tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea().posXPosYCorner(),
            tbots_cpp.Point(0.0, 0.0), tbots_cpp.Point(1, 0.0),
            tbots_cpp.Point(-3.5, 0.0), tbots_cpp.Vector(0.0, 0),
        ),
    ],
)
def test_crease_default(
        blue_bots,
        enemy_threat_position,
        yellow_bots,
        ball_initial_position,
        ball_initial_velocity,
        simulated_test_runner,
):

    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[blue_bots],
            yellow_robot_locations=[yellow_bots],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity
        ),
    )

    # These aren't necessary for this test, but this is just an example
    # of how to send commands to the simulator.
    #
    # NOTE: The gamecontroller responses are automatically handled by
    # the gamecontroller context manager class
    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].crease_defender.CopyFrom(
        CreaseDefenderTactic(
            enemy_threat_origin=tbots_cpp.createPointProto(enemy_threat_position),
            crease_defender_alignment=0,
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Always Validation
    always_validation_sequence_set = [
        [
            # RobotNeverEntersRegion(
            #     regions=[tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea()]
            # ),
            # BallNeverEntersRegion(
            #     regions=[tbots_cpp.Field.createSSLDivisionBField().friendlyGoal()]
            # ),
            NeverExcessivelyDribbles(),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            # Goalie should be in the defense area
            # RobotEventuallyEntersRegion(
            #     regions=[
            #         tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea()
            #     ]
            # ),
        ]
    ]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10
    )

#
# @pytest.mark.parametrize(
#     "ball_position,should_clear",
#     [
#         (
#                 tbots_cpp.Point(-3.45, 0),
#                 True,
#         ),  # ball is just inside the dead zone in the X direction
#         (
#                 tbots_cpp.Point(-3.45, 0.9),
#                 True,
#         ),  # ball is just inside the dead zone in the X direction
#         (
#                 tbots_cpp.Point(-4.0, 1.05),
#                 True,
#         ),  # ball is just inside the dead zone in the Y direction
#         (
#                 tbots_cpp.Point(0, 0),
#                 False
#                 # ball is just outside the dead zone in the X direction
#         ),
#     ],
# )
# def test_goalie_clears_from_dead_zone(
#         ball_position, should_clear, simulated_test_runner,
# ):
#     # Setup Robot
#     simulated_test_runner.simulator_proto_unix_io.send_proto(
#         WorldState,
#         create_world_state(
#             [],
#             blue_robot_locations=[
#                 tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea().centre()
#             ],
#             ball_location=ball_position,
#             ball_velocity=tbots_cpp.Vector(0, 0),
#         ),
#     )
#
#     # Setup Tactic
#     params = AssignedTacticPlayControlParams()
#     params.assigned_tactics[0].goalie.CopyFrom(
#         GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
#     )
#     simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
#         AssignedTacticPlayControlParams, params
#     )
#
#     # Setup no tactics on the enemy side
#     params = AssignedTacticPlayControlParams()
#     simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
#         AssignedTacticPlayControlParams, params
#     )
#
#     # Always Validation
#     always_validation_sequence_set = [
#         [
#             BallNeverEntersRegion(
#                 regions=[
#                     tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea()
#                 ]
#             )
#         ]
#     ]
#     if should_clear:
#         always_validation_sequence_set = [[]]
#
#     # Eventually Validation
#     eventually_validation_sequence_set = [[]]
#     if should_clear:
#         eventually_validation_sequence_set = [
#             [
#                 # Goalie should be in the defense area
#                 BallEventuallyExitsRegion(
#                     regions=[
#                         tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea()
#                     ]
#                 ),
#             ]
#         ]
#
#     simulated_test_runner.run_test(
#         test_timeout_s=8,
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         ag_always_validation_sequence_set=always_validation_sequence_set,
#     )


if __name__ == "__main__":
    pytest_main(__file__)
