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


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,position_to_block_from",
    [
        # Intercept straight line pass towards defender
        (tbots_cpp.Point(2, 0), tbots_cpp.Vector(-3, 0), tbots_cpp.Point(-2, 0)),
        # Intercept pass angled away from defender
        (tbots_cpp.Point(2, 0), tbots_cpp.Vector(-3, 0), tbots_cpp.Point(-2, 0.5)),
        # Intercept diagonal pass
        (
            tbots_cpp.Point(-1, -3),
            tbots_cpp.Vector(-2, 1.5),
            tbots_cpp.Point(-3, -0.75),
        ),
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
            position_to_block_from=tbots_cpp.createPointProto(position_to_block_from),
            ball_steal_mode=BallStealMode.STEAL,
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
                regions=[
                    tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea()
                ]
            ),
            # Defender should intercept ball and prevent it from entering the goal
            BallNeverEntersRegion(
                regions=[tbots_cpp.Field.createSSLDivisionBField().friendlyGoal()]
            ),
        ]
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
    "ball_initial_position,ball_initial_velocity,position_to_block_from",
    [
        # Place defender behind the ball and direct ball away from defender.
        # Defender should avoid/abort intercepting the ball in these scenarios.
        #  <---------O         X
        #           ball    defender
        (tbots_cpp.Point(-1, 0), tbots_cpp.Vector(-3, 0), tbots_cpp.Point(0, 0)),
        #  <---------O    ball
        #
        #            X    defender
        (tbots_cpp.Point(0, 0), tbots_cpp.Vector(-3, 0), tbots_cpp.Point(0, -1)),
        #            ^
        #            |
        #            |
        #            O    ball
        #
        #            X    defender
        (tbots_cpp.Point(0, 1), tbots_cpp.Vector(0, 3), tbots_cpp.Point(0, 0)),
    ],
)
def test_avoid_intercept_scenario(
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
            position_to_block_from=tbots_cpp.createPointProto(position_to_block_from),
            ball_steal_mode=BallStealMode.STEAL,
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
            # Defender should remain near position_to_block_from
            # and avoid/abort intercepting the pass, because either:
            #
            #  a) the ball is moving away from the defender and the
            #     defender cannot catch up
            #
            #  b) the ball has deviated significantly away from its
            #     initial pass trajectory
            #
            # Trying to continue intercepting the pass in these scenarios
            # leads to behaviours like ball chasing; this puts our defenders
            # out of position, creating open space on the field and allowing
            # the enemy team to set up again
            RobotAlwaysStaysInRegion(
                regions=[
                    tbots_cpp.Rectangle(
                        tbots_cpp.Point(
                            position_to_block_from.x() - 0.5,
                            position_to_block_from.y() - 0.5,
                        ),
                        tbots_cpp.Point(
                            position_to_block_from.x() + 0.5,
                            position_to_block_from.y() + 0.5,
                        ),
                    )
                ]
            )
        ]
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
    "ball_initial_position,ball_initial_velocity,position_to_block_from,enemy_kicker_position,should_steal",
    [
        # Intercept straight line pass towards defender, steal
        (
            tbots_cpp.Point(2, 0),
            tbots_cpp.Vector(-6, 0),
            tbots_cpp.Point(-2, 0),
            tbots_cpp.Point(2.2, 0),
            True,
        ),
        # Intercept pass angled away from defender, steal
        (
            tbots_cpp.Point(2, 0),
            tbots_cpp.Vector(-3, 0),
            tbots_cpp.Point(-2, 0.5),
            tbots_cpp.Point(2.2, 0),
            True,
        ),
        # Intercept diagonal pass, steal
        (
            tbots_cpp.Point(-1, -3),
            tbots_cpp.Vector(-2, 1.5),
            tbots_cpp.Point(-3, -0.75),
            tbots_cpp.Point(-0.8, 0),
            True,
        ),
        # Enemy too close, no steal
        (
            tbots_cpp.Point(0, 0),
            tbots_cpp.Vector(0, 0),
            tbots_cpp.Point(-1, 0),
            tbots_cpp.Point(0.2, 0),
            False,
        ),
        # Ball outside max range
        (
            tbots_cpp.Point(0, 0),
            tbots_cpp.Vector(0, 0),
            tbots_cpp.Point(-2, 0),
            tbots_cpp.Point(3, 0),
            False,
        ),
    ],
)
def test_steal_ball(
    ball_initial_position,
    ball_initial_velocity,
    position_to_block_from,
    enemy_kicker_position,
    should_steal,
    simulated_test_runner,
):
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            blue_robot_locations=[position_to_block_from],
            yellow_robot_locations=[enemy_kicker_position],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].pass_defender.CopyFrom(
        PassDefenderTactic(
            position_to_block_from=tbots_cpp.createPointProto(position_to_block_from),
            ball_steal_mode=BallStealMode.STEAL,
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
                regions=[
                    tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea()
                ]
            ),
            NeverExcessivelyDribbles(),
        ]
    ]  # Eventually Validation
    eventually_validation_sequence_set = [[]]

    if should_steal:
        eventually_validation_sequence_set = [
            [FriendlyEventuallyHasBallPossession(tolerance=0.05)]
        ]

    else:
        always_validation_sequence_set[0].append(
            FriendlyNeverHasBallPossession(tolerance=0.05)
        )

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
