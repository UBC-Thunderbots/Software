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
        (tbots_cpp.Point(0.25, 0.25), tbots_cpp.Point(0.80, 0.50), tbots_cpp.Angle.fromRadians(50), True),

        # Boundary Testing, because of the autoref implementation (initial of position Bot to final of Ball),
        # a conservative max dribble distance (0.95 m) is used

        # Test vertical dribbling
        (tbots_cpp.Point(0.01, 0), tbots_cpp.Point(0.96, 0), tbots_cpp.Angle(), True),

        # Test horizontal dribbling
        (tbots_cpp.Point(1, 1.5), tbots_cpp.Point(1.95, 1.5), tbots_cpp.Angle(), True),

        # Test bot and ball in same position
        (tbots_cpp.Point(0, 1), tbots_cpp.Point(0.95, 1), tbots_cpp.Angle(), True),
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

        # Boundary Testing, due to the conservative implementation a dribble distance of 1 m should fail

        # Test Vertical Dribbling
        (tbots_cpp.Point(0, 1), tbots_cpp.Point(0, 2), tbots_cpp.Angle(), True),

        # Test Horizontal Dribbling
        (tbots_cpp.Point(1, 2), tbots_cpp.Point(0, 2), tbots_cpp.Angle(), True),

        # Test Diagonal Dribbling
        (tbots_cpp.Point(0, 1), tbots_cpp.Point(0.6, 1.8), tbots_cpp.Angle(), True),

        # Test robot and ball at same position (affects dribbling orientation and therefore perceived dribble distance)
        (tbots_cpp.Point(0, 0), tbots_cpp.Point(0, 1), tbots_cpp.Angle(), True),
        (tbots_cpp.Point(0.0, 0.01), tbots_cpp.Point(0.81, 0.61), tbots_cpp.Angle(), True),
        (tbots_cpp.Point(0.01, 0.00), tbots_cpp.Point(0.81, 0.61), tbots_cpp.Angle(), True),
        (tbots_cpp.Point(0.0, 0.00), tbots_cpp.Point(0.81, 0.61), tbots_cpp.Angle(), True),
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
