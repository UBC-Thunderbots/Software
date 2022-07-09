import pytest

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
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


#target 0 = pos post
#target 1 = center
#target 2 = neg post

pos_post = tbots.Point((tbots.Field.createSSLDivisionBField().friendlyGoalpostPos() - tbots.Point(0,0.2)).x(), (tbots.Field.createSSLDivisionBField().friendlyGoalpostPos() - tbots.Point(0,0.2)).y())
neg_post = tbots.Point((tbots.Field.createSSLDivisionBField().friendlyGoalpostNeg() - tbots.Point(0, -0.2)).x(), (tbots.Field.createSSLDivisionBField().friendlyGoalpostNeg() - tbots.Point(0,-0.2)).y())
center_goal = tbots.Field.createSSLDivisionBField().friendlyGoalCenter()


def gen_test_case(ball_pos, ball_speeds, target, rob_pos):

    test_cases = []
    for ball_speed in ball_speeds:

        if target == 0:
            vel = (pos_post - ball_pos).normalize(ball_speed)
        elif target == 1:
            vel = (center_goal - ball_pos).normalize(ball_speed)
        elif target == 2:
            vel = (neg_post - ball_pos).normalize(ball_speed)

        test_cases.append((ball_pos, vel, rob_pos))

    return test_cases

# @pytest.mark.parametrize(
#     "ball_initial_position,ball_initial_velocity,robot_initial_position",
#     [
#         # test panic ball very fast in straight line
#         # (tbots.Point(-2, 0), tbots.Vector(-5, 0.75), tbots.Point(-4, 0)),
#
#         #from center shoot down
#         # *gen_test_case(tbots.Point(-2.5,0), [3,4,5], 2, center_goal),
#
#         #from up shoot near post
#         *gen_test_case(tbots.Point(-2.8, 2), [3,4,5], 0, center_goal),
#
#         # #from up shoot far post
#         # *gen_test_case(tbots.Point(-2.5,-2), [3,4,5], 2, center_goal),
#
#         # test panic ball very_fast in diagonal line
#         # TODO (#2609): failing tests when thunderscope is off
#         # (
#         #     tbots.Point(0, 0),
#         #     tbots.Vector(-5.5, 0.25),
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0, -0.5),
#         # ),
#         # test ball very fast misses net
#         # (tbots.Point(0, 0), tbots.Vector(-5, 1), tbots.Point(-4.5, 0)),
#         # test slow ball at sharp angle to friendly goal
#         # TODO (#2609): failing tests when thunderscope is off
#         # ball slow inside friendly defense area
#         # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 0)),
#         # # ball slow inside friendly defense area
#         # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 2)),
#         # # ball slow inside friendly defense area
#         # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 2)),
#         # ball slow inside friendly defense area
#         # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(-4, 0),),
#         # # ball stationary inside friendly defense area
#         # (
#         #     tbots.Point(-4, 0.0),
#         #     tbots.Vector(0.0, 0),
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalpostPos(),
#         # ),
#         # # ball stationary inside no-chip rectangle
#         # (
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0.1, 0.1),
#         #     tbots.Vector(-0.2, 0),
#         #     tbots.Point(-4, -1),
#         # ),
#         # # ball fast inside no-chip rectangle but no intersection with goal
#         # (
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0.1, 0),
#         #     tbots.Vector(0, -0.5),
#         #     tbots.Point(-3.5, 1),
#         # ),
#         # # ball moving out from inside defense area
#         # (
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0.5, 0),
#         #     tbots.Vector(0.5, 0),
#         #     tbots.Point(-3.5, 0),
#         # ),
#         # # ball slow inside no-chip rectangle
#         # (
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0.1, 0),
#         #     tbots.Vector(0.1, -0.1),
#         #     tbots.Point(-3.5, 1),
#         # ),
#         # # ball moving into goal from inside defense area
#         # (
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0.5, 0),
#         #     tbots.Vector(-0.5, 0),
#         #     tbots.Point(-3.5, 0),
#         # ),
#         # # ball moving up and out of defense area
#         # (
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0.3, 0),
#         #     tbots.Vector(0, 1),
#         #     tbots.Point(-3.5, 0),
#         # ),
#         # # ball moving down and out goal from defense area
#         # (
#         #     tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
#         #     + tbots.Vector(0.3, 0),
#         #     tbots.Vector(0, -0.7),
#         #     tbots.Point(-3.5, 0),
#         # ),
#     ],
# )
# def test_goalie_blocks_shot(
#     ball_initial_position,
#     ball_initial_velocity,
#     robot_initial_position,
#     simulated_test_runner,
# ):
#     # Setup Robot
#     simulated_test_runner.simulator_proto_unix_io.send_proto(
#         WorldState,
#         create_world_state(
#             [],
#             blue_robot_locations=[robot_initial_position],
#             ball_location=ball_initial_position,
#             ball_velocity=ball_initial_velocity,
#         ),
#     )
#
#     # These aren't necessary for this test, but this is just an example
#     # of how to send commands to the simulator.
#     #
#     # NOTE: The gamecontroller responses are automatically handled by
#     # the gamecontroller context manager class
#     simulated_test_runner.gamecontroller.send_ci_input(
#         gc_command=Command.Type.STOP, team=Team.UNKNOWN
#     )
#     simulated_test_runner.gamecontroller.send_ci_input(
#         gc_command=Command.Type.FORCE_START, team=Team.BLUE
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
#             RobotNeverEntersRegion(
#                 regions=[tbots.Field.createSSLDivisionBField().enemyDefenseArea()]
#             ),
#             # BallNeverEntersRegion(
#             #     regions=[tbots.Field.createSSLDivisionBField().friendlyGoal()]
#             # ),
#             NeverExcessivelyDribbles(),
#         ]
#     ]
#
#     # Eventually Validation
#     eventually_validation_sequence_set = [
#         [
#             # Goalie should be in the defense area
#             RobotEventuallyEntersRegion(
#                 regions=[tbots.Field.createSSLDivisionBField().friendlyDefenseArea()]
#             ),
#         ]
#     ]
#
#     simulated_test_runner.run_test(
#         eventually_validation_sequence_set=eventually_validation_sequence_set,
#         always_validation_sequence_set=always_validation_sequence_set,
#     )


def test_goalie_blocks_onetouch_shot(
        simulated_test_runner,
):

    print("in test")
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[center_goal],
            yellow_robot_locations=[tbots.Point(-2.8,-1)],
            ball_location=tbots.Point(-2.8,2),
            ball_velocity=tbots.Vector(0,-4),
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].goalie.CopyFrom(
        GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    ppoint = Point(x_meters=-2.5, y_meters=3)
    rpoint = Point(x_meters=-2.5, y_meters=-3)

    p = Pass(passer_point=ppoint, receiver_point=rpoint, pass_speed_m_per_s=3)

    params.assigned_tactics[0].receiver.CopyFrom(
        ReceiverTactic(r_pass=p, disable_one_touch_shot=False)
    )
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Always Validation
    always_validation_sequence_set = [
        [
            RobotNeverEntersRegion(
                regions=[tbots.Field.createSSLDivisionBField().enemyDefenseArea()]
            ),
            # BallNeverEntersRegion(
            #     regions=[tbots.Field.createSSLDivisionBField().friendlyGoal()]
            # ),
            NeverExcessivelyDribbles(),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            # Goalie should be in the defense area
            RobotEventuallyEntersRegion(
                regions=[tbots.Field.createSSLDivisionBField().friendlyDefenseArea()]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=7,
    )


if __name__ == "__main__":
    pytest_main(__file__)
