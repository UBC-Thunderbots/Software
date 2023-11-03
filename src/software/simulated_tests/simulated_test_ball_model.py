import pytest

import software.python_bindings as tbots_cpp
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_speed_threshold import *
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.ball_stops_in_region import *
from software.simulated_tests.excessive_dribbling import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.geometry_pb2 import Point, Angle
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)

# the friction model currently used in the er-force simulator

SLIDING_ACCELERATION = -3.432327  # equal to coeff_of_friction * g
ROLLING_ACCELERATION = -0.5
TRANSITION_FACTOR = 5.0 / 7.0
STOPPING_SPEED = 0.01


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity",
    [
        (tbots_cpp.Point(-3.5, 0), tbots_cpp.Vector(2, 0),),
        (tbots_cpp.Point(-3.5, 2), tbots_cpp.Vector(3, -2),),
        (tbots_cpp.Point(-3.5, -2), tbots_cpp.Vector(3, 2),),
        (tbots_cpp.Point(4.5, 3), tbots_cpp.Vector(-3.5, -2),),
    ],
)
def test_simulator_move_ball(
    ball_initial_position, ball_initial_velocity, simulated_test_runner,
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

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # expected ball position
    initial_v = ball_initial_velocity.length()

    # velocity at which ball starts to roll
    rolling_v = TRANSITION_FACTOR * initial_v
    time_until_roll = abs((rolling_v - initial_v) / SLIDING_ACCELERATION)
    time_until_stop = abs((rolling_v - STOPPING_SPEED) / ROLLING_ACCELERATION)

    d_slide = (initial_v + rolling_v) / 2 * time_until_roll
    d_roll = (rolling_v / 2) * time_until_stop
    total_distance = d_slide + d_roll

    ball_expected_position = (
        total_distance * ball_initial_velocity.normalize() + ball_initial_position
    )

    # Always Validation
    always_validation_sequence_set = [[NeverExcessivelyDribbles(),]]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [BallEventuallyStopsInRegion([tbots_cpp.Circle(ball_expected_position, 0.1)]),]
    ]

    simulated_test_runner.run_test(
        test_timeout_s=8,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
    )


def test_ball_robot_collision(simulated_test_runner):

    ball_initial_position = tbots_cpp.Field.createSSLDivisionBField().centerPoint()
    ball_initial_velocity = tbots_cpp.Vector(2.5, 0)
    robot_position = tbots_cpp.Point(2.5, 0)

    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[robot_position],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

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

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # expected ball position
    initial_v = ball_initial_velocity.length()

    # velocity at which ball starts to roll
    rolling_v = TRANSITION_FACTOR * initial_v
    time_until_roll = abs((rolling_v - initial_v) / SLIDING_ACCELERATION)
    time_until_stop = abs((rolling_v - STOPPING_SPEED) / ROLLING_ACCELERATION)

    d_slide = (initial_v + rolling_v) / 2 * time_until_roll
    d_roll = (rolling_v / 2) * time_until_stop
    total_distance = d_slide + d_roll

    # expected position if there was no robot collision
    ball_expected_position = (
        total_distance * ball_initial_velocity.normalize() + ball_initial_position
    )

    distance_from_robot = (ball_expected_position - robot_position).length()

    # Always Validation
    always_validation_sequence_set = []

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            BallEventuallyStopsInRegion(
                [tbots_cpp.Circle(robot_position, distance_from_robot)]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    pytest_main(__file__)
