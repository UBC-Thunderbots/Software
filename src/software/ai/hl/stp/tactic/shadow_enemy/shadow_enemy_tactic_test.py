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
from software.simulated_tests.ball_is_off_ground import *
from proto.message_translation.tbots_protobuf import create_world_state


@pytest.mark.parametrize(
    "blue_bots, yellow_bots, ball_initial_pos, ball_initial_velocity, attacker_direction, blue_final_destination",
    [
        # Test left and right align to centre crease
        (
            tbots_cpp.Point(-1, 0),
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(0.6, 0),
            tbots_cpp.Vector(0.1, 0),
            tbots_cpp.Angle.fromRadians(3.14),
            tbots_cpp.Point(1, 0),
        ),
    ],
)
def test_shadow_grab(
    blue_bots,
    yellow_bots,
    ball_initial_pos,
    ball_initial_velocity,
    attacker_direction,
    blue_final_destination,
    simulated_test_runner,
):
    # Setup Robot
    def setup(*args):
        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState,
            create_world_state(
                blue_robot_locations=[blue_bots],
                yellow_robot_locations=[yellow_bots],
                ball_location=ball_initial_pos,
                ball_velocity=ball_initial_velocity,
            ),
        )

        # Setup Tactic
        params = AssignedTacticPlayControlParams()

        params.assigned_tactics[0].shadow_enemy.CopyFrom(
            ShadowEnemyTactic(
                shadow_distance=0.0,
            )
        )

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, params
        )

        # Setup no tactics on the enemy side
        params = AssignedTacticPlayControlParams()

        params.assigned_tactics[0].dribble.CopyFrom(
            DribbleTactic(
                dribble_destination=tbots_cpp.createPointProto(blue_final_destination),
                final_dribble_orientation=tbots_cpp.createAngleProto(
                    attacker_direction
                ),
                allow_excessive_dribbling=False,
            )
        )

        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            AssignedTacticPlayControlParams, params
        )

    # Always Validation
    always_validation_sequence_set = [
        [
            RobotNeverEntersRegion(
                regions=[
                    tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea(),
                    tbots_cpp.Field.createSSLDivisionBField().enemyHalf(),
                    tbots_cpp.Circle(ball_initial_pos, 1),
                ]
            ),
            NeverExcessivelyDribbles(),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            # Crease defender should be near the defense area
            RobotEventuallyEntersRegion(
                regions=[
                    tbots_cpp.Field.createSSLDivisionBField()
                    .friendlyDefenseArea()
                    .expand(0.25)
                ]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
