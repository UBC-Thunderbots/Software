import pytest

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_stops_in_region import *
from software.simulated_tests.excessive_dribbling import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.geometry_pb2 import Point, Angle
from software.simulated_tests.tbots_test_fixture import *
from software.simulated_tests.pytest_main import pytest_main

# the friction model currently used in the er-force simulator

SLIDING_ACCELERATION = -3.432327  # equal to coeff_of_friction * g
ROLLING_ACCELERATION = -0.5
TRANSITION_FACTOR = 5.0 / 7.0
STOPPING_SPEED = 0.01


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity",
    [
        (tbots.Point(-3.5, 0), tbots.Vector(2, 0),),
        (tbots.Point(-3.5, 2), tbots.Vector(3, -2),),
        (tbots.Point(-3.5, -2), tbots.Vector(3, 2),),
        (tbots.Point(4.5, 3), tbots.Vector(-3.5, -2),),
    ],
)
@enable_thunderscope
def test_simulator_move_ball(
    ball_initial_position, ball_initial_velocity, simulated_test_runner,
):
    # Setup Ball
    simulated_test_runner.set_worldState(
        create_world_state(
            [],
            blue_robot_locations=[],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()

    simulated_test_runner.set_tactics(params, proto.ssl_gc_common_pb2.Team.BLUE)

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.set_tactics(params, proto.ssl_gc_common_pb2.Team.YELLOW)

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
        [BallEventuallyStopsInRegion([tbots.Circle(ball_expected_position, 0.1)]),]
    ]

    simulated_test_runner.run_test(
        test_timeout_s=8,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


@enable_thunderscope
def test_simulator_kick_ball(simulated_test_runner):

    ball_initial_position = tbots.Point(-2.5, 0)
    kick_velocity = tbots.Vector(2, 0)

    rob_pos = ball_initial_position - (kick_velocity.normalize() * 0.1)

    # Setup Ball
    simulated_test_runner.set_worldState(
        create_world_state(
            [],
            blue_robot_locations=[rob_pos],
            ball_location=ball_initial_position,
            ball_velocity=tbots.Vector(0, 0),
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    kick_origin = Point(
        x_meters=ball_initial_position.x(), y_meters=ball_initial_position.y()
    )

    params.assigned_tactics[0].kick.CopyFrom(
        KickTactic(
            kick_origin=kick_origin,
            kick_direction=Angle(radians=kick_velocity.orientation().toRadians()),
            kick_speed_meters_per_second=kick_velocity.length(),
        )
    )

    simulated_test_runner.set_tactics(params, proto.ssl_gc_common_pb2.Team.BLUE)

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.set_tactics(params, proto.ssl_gc_common_pb2.Team.YELLOW)

    # expected ball position

    initial_v = kick_velocity.length()

    # velocity at which ball starts to roll
    rolling_v = TRANSITION_FACTOR * initial_v
    time_until_roll = abs((rolling_v - initial_v) / SLIDING_ACCELERATION)
    time_until_stop = abs((rolling_v - STOPPING_SPEED) / ROLLING_ACCELERATION)

    d_slide = (initial_v + rolling_v) / 2 * time_until_roll
    d_roll = (rolling_v / 2) * time_until_stop
    total_distance = d_slide + d_roll

    ball_expected_position = (
        total_distance * kick_velocity.normalize() + ball_initial_position
    )

    # Always Validation
    always_validation_sequence_set = []

    # Eventually Validation
    # use larger error margin because we can't account for robot-ball collision impact
    eventually_validation_sequence_set = [
        [BallEventuallyStopsInRegion([tbots.Circle(ball_expected_position, 0.5)]),]
    ]

    simulated_test_runner.run_test(
        test_timeout_s=8,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


@enable_thunderscope
def test_ball_robot_collision(simulated_test_runner):

    ball_initial_position = tbots.Field.createSSLDivisionBField().centerPoint()
    ball_initial_velocity = tbots.Vector(2.5, 0)
    robot_position = tbots.Point(2.5, 0)

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
                [tbots.Circle(robot_position, distance_from_robot)]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=4,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    pytest_main(__file__)
