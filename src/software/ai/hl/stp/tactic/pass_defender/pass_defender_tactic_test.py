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


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,position_to_block_from",
    [
        # Intercept straight line pass towards defender
        (tbots.Point(2, 0), tbots.Vector(-3, 0), tbots.Point(-2, 0)),
        # Intercept pass angled away from defender
        (tbots.Point(2, 0), tbots.Vector(-3, 0), tbots.Point(-2, 0.5)),
        # Intercept diagonal pass
        (tbots.Point(-1, -3), tbots.Vector(-2, 1.5), tbots.Point(-3, -0.75)),
    ],
)
def test_ball_chipped_on_intercept(
    ball_initial_position,
    ball_initial_velocity,
    position_to_block_from,
    simulated_test_runner,
):
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[position_to_block_from],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].pass_defender.CopyFrom(
        PassDefenderTactic(
            position_to_block_from=tbots.createPointProto(position_to_block_from)
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
        [
            RobotNeverEntersRegion(
                regions=[tbots.Field.createSSLDivisionBField().friendlyDefenseArea()]
            ),
            # Defender should intercept ball and prevent it from entering the goal
            BallNeverEntersRegion(
                regions=[tbots.Field.createSSLDivisionBField().friendlyGoal()]
            ),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=25,
    )


if __name__ == "__main__":
    pytest_main(__file__)
