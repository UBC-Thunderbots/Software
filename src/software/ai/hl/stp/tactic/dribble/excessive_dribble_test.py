import pytest

import software.python_bindings as tbots_cpp
from software.simulated_tests.excessive_dribbling import (
    NeverExcessivelyDribbles,
    EventuallyStartsExcessivelyDribbling,
)
from proto.message_translation.tbots_protobuf import (
    WorldState,
    AssignedTacticPlayControlParams,
    DribbleTactic,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state


@pytest.mark.parametrize(
    "initial_location,dribble_destination,final_dribble_orientation,allow_excessive_dribbling, should_excessively_dribble",
    [
        # Tests Should not excessively dribble
        # Dribble Destination for the ball < 1.0 from its starting position
        (
            tbots_cpp.Point(0.5, 0),
            tbots_cpp.Point(1.02, 0),
            tbots_cpp.Angle(),
            True,
            False,
        ),
        # Dribble Testing diagonally
        (
            tbots_cpp.Point(0.25, 0.25),
            tbots_cpp.Point(0.80, 0.50),
            tbots_cpp.Angle.fromRadians(50),
            True,
            False,
        ),
        # Boundary Testing, because of the autoref implementation (initial of position Bot to final of Ball),
        # a conservative max dribble distance (0.95 m) is used
        # Test vertical dribbling
        (
            tbots_cpp.Point(0.01, 0),
            tbots_cpp.Point(0.96, 0),
            tbots_cpp.Angle(),
            True,
            False,
        ),
        # Test horizontal dribbling
        (
            tbots_cpp.Point(1, 1.5),
            tbots_cpp.Point(1.95, 1.5),
            tbots_cpp.Angle(),
            True,
            False,
        ),
        # Test bot and ball in same position
        (
            tbots_cpp.Point(0, 1),
            tbots_cpp.Point(0.95, 1),
            tbots_cpp.Angle(),
            True,
            False,
        ),
        # Tests Should excessively dribble
        # Dribble Destination for the ball > 1.0 from its starting position
        (tbots_cpp.Point(0, 2), tbots_cpp.Point(0, 0.5), tbots_cpp.Angle(), True, True),
        # Dribble Testing diagonally
        (
            tbots_cpp.Point(0.1, 1.1),
            tbots_cpp.Point(1.1, 0.1),
            tbots_cpp.Angle.fromRadians(50),
            True,
            True,
        ),
        # Boundary Testing, due to the conservative implementation a dribble distance of 1 m should fail
        # Test Vertical Dribbling
        (tbots_cpp.Point(0, 1), tbots_cpp.Point(0, 2), tbots_cpp.Angle(), True, True),
        # Test Horizontal Dribbling
        (tbots_cpp.Point(1, 2), tbots_cpp.Point(0, 2), tbots_cpp.Angle(), True, True),
        # Test Diagonal Dribbling
        (
            tbots_cpp.Point(0, 1),
            tbots_cpp.Point(0.6, 1.8),
            tbots_cpp.Angle(),
            True,
            True,
        ),
        # Test robot and ball at same position (affects dribbling orientation and therefore perceived dribble distance)
        (tbots_cpp.Point(0, 0), tbots_cpp.Point(0, 1), tbots_cpp.Angle(), True, True),
        (
            tbots_cpp.Point(0.0, 0.01),
            tbots_cpp.Point(0.81, 0.61),
            tbots_cpp.Angle(),
            True,
            True,
        ),
        (
            tbots_cpp.Point(0.01, 0.00),
            tbots_cpp.Point(0.81, 0.61),
            tbots_cpp.Angle(),
            True,
            True,
        ),
    ],
)
def test_excessive_dribbling(
    initial_location,
    dribble_destination,
    final_dribble_orientation,
    allow_excessive_dribbling,
    should_excessively_dribble,
    simulated_test_runner,
):
    if should_excessively_dribble:
        blue_robot_locations = [tbots_cpp.Point(0, 0.0)]

        # Always and Eventually validation sets for excessive dribbling
        always_validation_sequence_set = [[]]
        eventually_validation_sequence_set = [[EventuallyStartsExcessivelyDribbling()]]
    else:
        blue_robot_locations = [tbots_cpp.Point(0, 1)]

        # Always and Eventually validation sets for not excessive dribbling
        always_validation_sequence_set = [[NeverExcessivelyDribbles()]]
        eventually_validation_sequence_set = [[]]

    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=blue_robot_locations,
            ball_location=initial_location,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].dribble.CopyFrom(
        DribbleTactic(
            dribble_destination=tbots_cpp.createPointProto(dribble_destination),
            final_dribble_orientation=tbots_cpp.createAngleProto(
                final_dribble_orientation
            ),
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

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
