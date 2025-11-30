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
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team

@pytest.mark.parametrize(
    "initial_location,dribble_destination,final_dribble_orientation,allow_excessive_dribbling",
    [
        # Dribble Destination for the ball < 1.0 from its starting position
        (tbots_cpp.Point(0.5, 0), tbots_cpp.Point(1.02, 0), tbots_cpp.Angle(), True),

        # Dribble Testing diagonally
        (tbots_cpp.Point(0.25, 0.25), tbots_cpp.Point(0.85, 0.65), tbots_cpp.Angle.fromRadians(50), True),

        # Boundary Testing, because of the autoref implementation (initial of position Bot to final of Ball),
        # a dribble distance a tiny bit over 1m should pass
        (tbots_cpp.Point(0.01, 0), tbots_cpp.Point(1.025, 0), tbots_cpp.Angle(), True),
        (tbots_cpp.Point(1, 1.5), tbots_cpp.Point(2, 1.5), tbots_cpp.Angle(), True),

        # When the robot is directly on the ball, the orientation at which the bot begins to
        # dribble is different; there appears to be a bit more leeway
        (tbots_cpp.Point(0, 1), tbots_cpp.Point(0, 2.025), tbots_cpp.Angle(), True),
    ],
)

def test_not_excessive_dribbling(
        initial_location,
        dribble_destination,
        final_dribble_orientation,
        allow_excessive_dribbling,
        simulated_test_runner,
):
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[tbots_cpp.Point(0, 1)],
            ball_location=initial_location,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )


    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].dribble.CopyFrom(
        DribbleTactic(
            dribble_destination=tbots_cpp.createPointProto(dribble_destination),
            final_dribble_orientation=tbots_cpp.createAngleProto(final_dribble_orientation),
            allow_excessive_dribbling=allow_excessive_dribbling,
        )
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
        [NeverExcessivelyDribbles()]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
    )


@pytest.mark.parametrize(
    "initial_location,dribble_destination,final_dribble_orientation,allow_excessive_dribbling",
    [
        # Dribble Destination for the ball > 1.0 from its starting position
        (tbots_cpp.Point(0, 2), tbots_cpp.Point(0, 0.5), tbots_cpp.Angle(), True),
        # Dribble Testing diagonally
        (tbots_cpp.Point(0.1, 1.1), tbots_cpp.Point(1.1, 0.1), tbots_cpp.Angle.fromRadians(50), True),
        # Boundary Testing, because of the autoref implementation (initial position of Bot to final of Ball),
        # a dribble distance a tiny bit over 1m should pass, but too much (> 1.015 from testing) fails
        (tbots_cpp.Point(0, 1), tbots_cpp.Point(0, 2.02), tbots_cpp.Angle(), True),

        # When the robot is directly on the ball, the orientation at which the bot begins to
        # dribble is different; there appears to be a bit more leeway on the distance that passes
        # Furthermore, that leeway appears to be different depending on the position of the bot and ball
        # (e.g. here the point is 0, 0, and the leeway is 1 cm more than at 0, 1)
        (tbots_cpp.Point(0, 0), tbots_cpp.Point(0, 1.035), tbots_cpp.Angle(), True),
    ],
)

def test_excessive_dribbling(
        initial_location,
        dribble_destination,
        final_dribble_orientation,
        allow_excessive_dribbling,
        simulated_test_runner,
):
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[tbots_cpp.Point(0, 0.0)],
            ball_location=initial_location,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )


    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].dribble.CopyFrom(
        DribbleTactic(
            dribble_destination=tbots_cpp.createPointProto(dribble_destination),
            final_dribble_orientation=tbots_cpp.createAngleProto(final_dribble_orientation),
            allow_excessive_dribbling=allow_excessive_dribbling,
        )
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
    always_validation_sequence_set = [[]]

    # Eventually Validation
    eventually_validation_sequence_set = [[EventuallyStartsExcessivelyDribbling()]]

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
