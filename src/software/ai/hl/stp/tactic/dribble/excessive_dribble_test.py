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
    "dribble_destination,final_dribble_orientation,allow_excessive_dribbling",
    [
        # Dribble Destination for the ball > 1.0 from its starting position
        (tbots_cpp.Point(1.012, 0), tbots_cpp.Angle(), True),
    ],
)

def test_excessive_dribbling(
        dribble_destination,
        final_dribble_orientation,
        allow_excessive_dribbling,
        simulated_test_runner,
):
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[tbots_cpp.Point(1, 0)],
            ball_location=tbots_cpp.Point(0, 0),
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
        [BallAlwaysStaysInRegion(
        regions=[tbots_cpp.Circle(tbots_cpp.Point(0, 0), 1)]
        )],
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


if __name__ == "__main__":
    pytest_main(__file__)
