import sys

import pytest
import itertools

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_speed_threshold import *
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


# @pytest.mark.parametrize(
#     "ball_initial_position,ball_initial_velocity",
#     [
#         (
#             tbots.Point(-3.5, 0),
#             tbots.Vector(2, 0),
#         ),
#         (
#             tbots.Point(-3.5, 2),
#             tbots.Vector(3, -2),
#         ),
#         (
#             tbots.Point(-3.5, -2),
#             tbots.Vector(3, 2),
#         ),
#         (
#             tbots.Point(4.5, 3),
#             tbots.Vector(-3.5, -2),
#         ),
#     ],
# )
#
# def test_simulator_move_ball(
#         ball_initial_position,
#         ball_initial_velocity,
#         simulated_test_runner,
# ):
#     # Setup Ball
#     simulated_test_runner.simulator_proto_unix_io.send_proto(
#         WorldState,
#         create_world_state(
#             [],
#             blue_robot_locations=[],
#             ball_location=ball_initial_position,
#             ball_velocity=ball_initial_velocity,
#         ),
#     )
#
#     # Setup Tactic
#     params = AssignedTacticPlayControlParams()
#
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
#     #expected ball position
#
#     transition_factor = 5.0/7.0
#
#     # equal to coeff_of_friction * g
#     sliding_a = -3.4323275
#
#     rolling_a = -0.5
#     initial_v = ball_initial_velocity.length()
#
#     #velocity at which ball starts to roll
#     rolling_v = transition_factor * initial_v
#     time_until_roll = abs((rolling_v - initial_v)/sliding_a)
#     time_until_stop = abs((rolling_v - 0.01)/ rolling_a)
#
#     d_slide = (initial_v+rolling_v)/2 * time_until_roll
#     d_roll = (rolling_v/2) * time_until_stop
#     total_distance = d_slide + d_roll
#
#     ball_expected_position = total_distance * ball_initial_velocity.normalize() + ball_initial_position
#
#     # Always Validation
#     always_validation_sequence_set = [
#         [
#             NeverExcessivelyDribbles(),
#         ]
#     ]
#
#     # Eventually Validation
#     eventually_validation_sequence_set = [
#         [
#             BallEntersRegion([tbots.Circle(ball_expected_position, 0.1)]),
#         ]
#     ]
#
#     simulated_test_runner.run_test(
#         test_timeout_s=6,
#         eventually_validation_sequence_set=eventually_validation_sequence_set,
#         always_validation_sequence_set=always_validation_sequence_set,
#     )

@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity",
    [
        (
                tbots.Point(-3.5, 0),
                tbots.Vector(2, 0),
        ),
        (
                tbots.Point(-3.5, 2),
                tbots.Vector(3, -2),
        ),
        (
                tbots.Point(-3.5, -2),
                tbots.Vector(3, 2),
        ),
        (
                tbots.Point(4.5, 3),
                tbots.Vector(-3.5, -2),
        ),
    ],
)

def test_simulator_kick_ball(
        ball_initial_position,
        ball_initial_velocity,
        simulated_test_runner,
):
    # Setup Ball
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].kick.CopyFrom(
        KickTactic(kick_origin=ball_initial_position, kick_direction=tbots.Angle.fromDegrees(0.0), kick_speed_meters_per_second=ball_initial_velocity.length())
    )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    #expected ball position

    transition_factor = 5.0/7.0

    # equal to coeff_of_friction * g
    sliding_a = -3.4323275

    rolling_a = -0.5
    initial_v = ball_initial_velocity.length()

    #velocity at which ball starts to roll
    rolling_v = transition_factor * initial_v
    time_until_roll = abs((rolling_v - initial_v)/sliding_a)
    time_until_stop = abs((rolling_v - 0.01)/ rolling_a)

    d_slide = (initial_v+rolling_v)/2 * time_until_roll
    d_roll = (rolling_v/2) * time_until_stop
    total_distance = d_slide + d_roll

    ball_expected_position = total_distance * ball_initial_velocity.normalize() + ball_initial_position

    # Always Validation
    always_validation_sequence_set = [
        [
            NeverExcessivelyDribbles(),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            BallEntersRegion([tbots.Circle(ball_expected_position, 0.1)]),
        ]
    ]

    simulated_test_runner.run_test(
        test_timeout_s=6,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


# def test_ball_robot_collision(simulated_test_runner):
#
#     ball_initial_position = tbots.Field.createSSLDivisionBField().centerPoint()
#     ball_initial_velocity = tbots.Vector(2.5, 0)
#     robot_position = tbots.Point(2.5,0)
#
#     # Setup Robot
#     simulated_test_runner.simulator_proto_unix_io.send_proto(
#         WorldState,
#         create_world_state(
#             [],
#             blue_robot_locations=[robot_position],
#             ball_location=ball_initial_position,
#             ball_velocity=ball_initial_velocity,
#         ),
#     )
#
#     # Setup Ball
#     simulated_test_runner.simulator_proto_unix_io.send_proto(
#         WorldState,
#         create_world_state(
#             [],
#             blue_robot_locations=[],
#             ball_location=ball_initial_position,
#             ball_velocity=ball_initial_velocity,
#         ),
#     )
#
#     # Setup Tactic
#     params = AssignedTacticPlayControlParams()
#
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
#             NeverExcessivelyDribbles(),
#         ]
#     ]
#
#     # Eventually Validation
#     eventually_validation_sequence_set = [
#         [
#             BallEntersRegion([tbots.Circle(robot_position, 1)]),
#         ]
#     ]
#
#     simulated_test_runner.run_test(
#         eventually_validation_sequence_set=eventually_validation_sequence_set,
#         always_validation_sequence_set=always_validation_sequence_set,
#     )
#



if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
