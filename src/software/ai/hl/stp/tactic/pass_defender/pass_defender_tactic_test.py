import pytest

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.test_constants import *
from software.simulated_tests.simulated_test_fixture import pytest_main
from proto.message_translation.tbots_protobuf import create_world_state


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,position_to_block_from",
    [
        # Intercept straight line pass towards defender
        # (tbots.Point(2, 0), tbots.Vector(-3, 0), tbots.Point(-2, 0)),
        # Intercept pass angled away from defender
        # (tbots.Point(2, 0), tbots.Vector(-3, 0), tbots.Point(-2, 0.5)),
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
    eventually_validation_sequence_set = [
        [
            BallEventuallyEntersRegion(regions=[FIELD_RIGHT_HALF]),
            BallEventuallyMovesToFromRobot(robot_id=0, to_robot=False),
        ]
    ]

    simulated_test_runner.run_test(
        test_timeout_s=20,
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
        (tbots.Point(-1, 0), tbots.Vector(-3, 0), tbots.Point(0, 0)),
        #  <---------O    ball
        #
        #            X    defender
        (tbots.Point(0, 0), tbots.Vector(-3, 0), tbots.Point(0, -1)),
        #            ^
        #            |
        #            |
        #            O    ball
        #
        #            X    defender
        (tbots.Point(0, 1), tbots.Vector(0, 3), tbots.Point(0, 0)),
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
                    tbots.Rectangle(
                        tbots.Point(
                            position_to_block_from.x() - 0.5,
                            position_to_block_from.y() - 0.5,
                        ),
                        tbots.Point(
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
        test_timeout_s=20,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
