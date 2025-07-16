import pytest

import software.python_bindings as tbots_cpp
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.ball_enters_region import BallEventuallyEntersRegion, BallAlwaysStaysInRegion
from software.simulated_tests.ball_moves_forward import BallAlwaysMovesForward
from software.simulated_tests.friendly_has_ball_possession import FriendlyAlwaysHasBallPossession


def test_penalty_kick_no_goalie(simulated_test_runner):
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=[],
            blue_robot_locations=[tbots_cpp.Point(-3, 0)],
            ball_location=tbots_cpp.Point(0,0),
            ball_velocity=0
        )
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].penalty_kick.CopyFrom(
        PenaltyKickTactic()
    )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlPArams, params
    )

    # Validation
    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=[[BallEventuallyEntersRegion(regions=[tbots_cpp.Field.createSSLDivisionBField().enemyGoal()])]],
        inv_always_validation_sequence_set=[[BallAlwaysStaysInRegion(regions=[tbots_cpp.Field.createSSLDivisionBField().fieldLines()]),
                                             FriendlyAlwaysHasBallPossession(), BallAlwaysMovesForward(tbots_cpp.Point(0,0))]],
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10
    )
