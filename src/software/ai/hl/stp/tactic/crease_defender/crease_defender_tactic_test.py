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
    simulated_test_runner,
    pytest_main,
)
from software.simulated_tests.ball_is_off_ground import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "blue_bots, yellow_bots, ball_initial_pos, ball_initial_velocity",
    [
        # Test left and right align to centre crease
        (
            tbots_cpp.Field.createSSLDivisionBField()
            .friendlyDefenseArea()
            .posXPosYCorner()
            + tbots_cpp.Vector(0.5, 0),
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Vector(0, 0),
        ),
        (
            tbots_cpp.Field.createSSLDivisionBField()
            .friendlyDefenseArea()
            .posXNegYCorner()
            + tbots_cpp.Vector(0.5, 0),
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Vector(0, 0),
        ),
    ],
)
def test_crease_positioning(
    blue_bots,
    yellow_bots,
    ball_initial_pos,
    ball_initial_velocity,
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

        params.assigned_tactics[0].crease_defender.CopyFrom(
            CreaseDefenderTactic(
                enemy_threat_origin=tbots_cpp.createPointProto(ball_initial_pos),
                crease_defender_alignment=CreaseDefenderAlignment.CENTRE,
                max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
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
        test_timeout_s=4,
    )


friendly_defense_area_front_center = tbots_cpp.Point(
    tbots_cpp.Field.createSSLDivisionBField()
    .friendlyDefenseArea()
    .posXPosYCorner()
    .x(),
    tbots_cpp.Field.createSSLDivisionBField().friendlyDefenseArea().centre().y(),
)


@pytest.mark.parametrize(
    "blue_bots, yellow_bots, ball_initial_pos, ball_initial_velocity, should_chip",
    [
        # Test auto chip over enemy
        (
            friendly_defense_area_front_center + tbots_cpp.Vector(0.5, 0),
            friendly_defense_area_front_center + tbots_cpp.Vector(1.1, 0),
            friendly_defense_area_front_center + tbots_cpp.Vector(0.9, 0),
            tbots_cpp.Vector(-2, 0),
            True,
        ),
        # Test block, auto chip off, enemy far
        (
            friendly_defense_area_front_center + tbots_cpp.Vector(0.5, 0),
            friendly_defense_area_front_center + tbots_cpp.Vector(5, 0),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Vector(-6, 0),
            False,
        ),
        # Test block, auto chip off, enemy close, facing net
        (
            friendly_defense_area_front_center + tbots_cpp.Vector(1.5, 0),
            friendly_defense_area_front_center + tbots_cpp.Vector(0.2, 0),
            friendly_defense_area_front_center + tbots_cpp.Vector(0.9, 0),
            tbots_cpp.Vector(2, 0),
            False,
        ),
        # Test block, auto chip off, enemy far, facing net
        (
            friendly_defense_area_front_center + tbots_cpp.Vector(3, 0),
            tbots_cpp.Point(3, 0),
            friendly_defense_area_front_center + tbots_cpp.Vector(0.9, 0),
            tbots_cpp.Vector(2, 0),
            False,
        ),
    ],
)
def test_crease_autochip(
    blue_bots,
    yellow_bots,
    ball_initial_pos,
    ball_initial_velocity,
    should_chip,
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

        params.assigned_tactics[0].crease_defender.CopyFrom(
            CreaseDefenderTactic(
                enemy_threat_origin=tbots_cpp.createPointProto(ball_initial_pos),
                crease_defender_alignment=CreaseDefenderAlignment.CENTRE,
                max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
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
            BallIsAlwaysOnGround(threshold=0.05),
            NeverExcessivelyDribbles(),
        ]
    ]
    # Eventually Validation
    eventually_validation_sequence_set = [[]]

    if should_chip:
        # Always Validation for chipping
        always_validation_sequence_set = [[]]
        # Eventually Validation for chipping
        eventually_validation_sequence_set = [
            [BallIsEventuallyOffGround(threshold=0.05)]
        ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=3,
    )


@pytest.mark.parametrize(
    "blue_bots, yellow_bots, ball_initial_pos, ball_initial_velocity, should_dribble",
    [
        # Test get ball in range
        (
            friendly_defense_area_front_center + tbots_cpp.Vector(0.5, 0),
            tbots_cpp.Point(0, 0),
            friendly_defense_area_front_center + tbots_cpp.Vector(1, 0),
            tbots_cpp.Vector(0, 0),
            True,
        ),
        # Test leave ball out of range
        (
            friendly_defense_area_front_center + tbots_cpp.Vector(0.5, 0),
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0, 0) + tbots_cpp.Vector(-0.5, 0),
            tbots_cpp.Vector(0, 0),
            False,
        ),
    ],
)
def test_crease_get_ball(
    blue_bots,
    yellow_bots,
    ball_initial_pos,
    ball_initial_velocity,
    should_dribble,
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

        params.assigned_tactics[0].crease_defender.CopyFrom(
            CreaseDefenderTactic(
                enemy_threat_origin=tbots_cpp.createPointProto(ball_initial_pos),
                crease_defender_alignment=CreaseDefenderAlignment.CENTRE,
                max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
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
            BallIsAlwaysOnGround(threshold=0.05),
            NeverExcessivelyDribbles(),
        ]
    ]
    # Eventually Validation
    eventually_validation_sequence_set = [[]]

    if should_dribble:
        eventually_validation_sequence_set = [
            [
                RobotEventuallyEntersRegion(
                    regions=[tbots_cpp.Circle(ball_initial_pos, 0.2)]
                )
            ]
        ]
    else:
        always_validation_sequence_set[0].append(
            RobotNeverEntersRegion(regions=[tbots_cpp.Circle(ball_initial_pos, 0.2)])
        )

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=3,
    )


if __name__ == "__main__":
    pytest_main(__file__)
