import pytest

import software.python_bindings as tbots_cpp
from proto.import_all_protos import (
    CreaseDefenderTactic,
    CreaseDefenderAlignment,
    MaxAllowedSpeedMode,
    BallStealMode,
)
from software.simulated_tests.pytest_validations.robot_enters_region import *
from software.simulated_tests.pytest_validations.ball_enters_region import *
from software.simulated_tests.pytest_validations.ball_moves_in_direction import *
from software.simulated_tests.pytest_validations.friendly_has_ball_possession import *
from software.simulated_tests.pytest_validations.ball_speed_threshold import *
from software.simulated_tests.pytest_validations.robot_speed_threshold import *
from software.simulated_tests.pytest_validations.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from software.simulated_tests.pytest_validations.ball_is_off_ground import *
from proto.message_translation.tbots_protobuf import create_world_state


def test_not_bumping_ball_towards_net(simulated_test_runner):
    enemy_threat_point = tbots_cpp.Point(3, 0)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[tbots_cpp.Point(0, 0)],
                yellow_robot_locations=[tbots_cpp.Point(4, 0)],
                ball_location=enemy_threat_point,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: CreaseDefenderTactic(
                    enemy_threat_origin=tbots_cpp.createPointProto(enemy_threat_point),
                    crease_defender_alignment=CreaseDefenderAlignment.CENTRE,
                )
            }
        )

    simulated_test_runner.run_test(
        setup=setup,
        inv_always_validation_sequence_set=[[BallSpeedAlwaysBelowThreshold(0.001)]],
        ag_always_validation_sequence_set=[[BallSpeedAlwaysBelowThreshold(0.001)]],
    )


@pytest.mark.parametrize(
    "enemy_threat_point,crease_alignment,region_index",
    [
        # Enemy threat in front of crease, LEFT
        (tbots_cpp.Point(1, 2.5), CreaseDefenderAlignment.LEFT, 2),
        # Enemy threat in front of crease, CENTRE
        (tbots_cpp.Point(1, -2.5), CreaseDefenderAlignment.CENTRE, 3),
        # Enemy threat in front of crease, RIGHT
        (tbots_cpp.Point(1.5, 2), CreaseDefenderAlignment.RIGHT, 2),
        # Enemy threat left side of crease, RIGHT
        (tbots_cpp.Point(-3.5, 2.5), CreaseDefenderAlignment.RIGHT, 1),
        # Enemy threat left side of crease, CENTRE
        (tbots_cpp.Point(-4, 2.5), CreaseDefenderAlignment.CENTRE, 0),
        # goal Enemy threat right side of crease, RIGHT
        (tbots_cpp.Point(-4, -2), CreaseDefenderAlignment.RIGHT, 5),
        # Enemy threat right side of crease, LEFT
        (tbots_cpp.Point(-4.25, -2), CreaseDefenderAlignment.LEFT, 5),
    ],
)
def test_crease_region_positioning(
    enemy_threat_point, crease_alignment, region_index, simulated_test_runner
):
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[tbots_cpp.Point(-3, 1.5)],
                yellow_robot_locations=[
                    tbots_cpp.Point(1, 0),
                    enemy_threat_point,
                    tbots_cpp.Point(1, -1.5),
                    field.enemyGoalCenter(),
                    field.enemyDefenseArea().negXNegYCorner(),
                    field.enemyDefenseArea().negXPosYCorner(),
                ],
                ball_location=tbots_cpp.Point(4.5, -3),
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: CreaseDefenderTactic(
                    enemy_threat_origin=tbots_cpp.createPointProto(enemy_threat_point),
                    crease_defender_alignment=crease_alignment,
                )
            }
        )

    # We check if the robot is in one of the following regions
    #  ┌───┬───┬────────────────┐
    #  │   │   │                │
    #  │ 0 │ 1 │                C
    #  │   │   │                E
    #  │   │   │                N
    #  ├───┴───┤       2        T
    #  E       │                R
    #  N  Def. │                E
    #  D       │                │
    #  L       ├────────────────┤
    #  I  Area │                │
    #  N       │                │
    #  E       │                L
    #  ├───┬───┤                I
    #  │   │   │       3        N
    #  │ 5 │ 4 │                E
    #  │   │   │                │
    #  │   │   │                │
    #  └───┴───┴────────────────┘

    defense_area = field.friendlyDefenseArea()
    field_lines = field.fieldLines()

    defense_area_half_width = tbots_cpp.Vector(defense_area.xLength() / 2, 0)

    defender_regions = [
        tbots_cpp.Rectangle(
            field.friendlyCornerPos(),
            defense_area.negXPosYCorner() + defense_area_half_width,
        ),
        tbots_cpp.Rectangle(
            field.friendlyCornerPos() + defense_area_half_width,
            defense_area.posXPosYCorner(),
        ),
        tbots_cpp.Rectangle(
            tbots_cpp.Point(defense_area.xMax(), field_lines.yMax()),
            field.centerPoint(),
        ),
        tbots_cpp.Rectangle(
            tbots_cpp.Point(defense_area.xMax(), field_lines.yMin()),
            field.centerPoint(),
        ),
        tbots_cpp.Rectangle(
            field.friendlyCornerNeg() + defense_area_half_width,
            defense_area.posXNegYCorner(),
        ),
        tbots_cpp.Rectangle(
            field.friendlyCornerNeg(),
            defense_area.negXNegYCorner() + defense_area_half_width,
        ),
    ]

    eventually_validations = [
        [
            # TODO: check that it stays for 1000 ticks
            RobotEventuallyEntersRegion(regions=[defender_regions[region_index]])
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
    )


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
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[blue_bots],
                yellow_robot_locations=[yellow_bots],
                ball_location=ball_initial_pos,
                ball_velocity=ball_initial_velocity,
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: CreaseDefenderTactic(
                    enemy_threat_origin=tbots_cpp.createPointProto(ball_initial_pos),
                    crease_defender_alignment=CreaseDefenderAlignment.CENTRE,
                    max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                    ball_steal_mode=BallStealMode.STEAL,
                )
            }
        )

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

    eventually_validation_sequence_set = [
        [
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
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[blue_bots],
                yellow_robot_locations=[yellow_bots],
                ball_location=ball_initial_pos,
                ball_velocity=ball_initial_velocity,
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: CreaseDefenderTactic(
                    enemy_threat_origin=tbots_cpp.createPointProto(ball_initial_pos),
                    crease_defender_alignment=CreaseDefenderAlignment.CENTRE,
                    max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                    ball_steal_mode=BallStealMode.STEAL,
                )
            }
        )

    always_validation_sequence_set = [
        [
            BallIsAlwaysOnGround(),
            NeverExcessivelyDribbles(),
        ]
    ]
    eventually_validation_sequence_set = [[]]

    if should_chip:
        always_validation_sequence_set = [[]]
        eventually_validation_sequence_set = [[BallIsEventuallyOffGround()]]

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
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[blue_bots],
                yellow_robot_locations=[yellow_bots],
                ball_location=ball_initial_pos,
                ball_velocity=ball_initial_velocity,
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: CreaseDefenderTactic(
                    enemy_threat_origin=tbots_cpp.createPointProto(ball_initial_pos),
                    crease_defender_alignment=CreaseDefenderAlignment.CENTRE,
                    max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                    ball_steal_mode=BallStealMode.STEAL,
                )
            }
        )

    always_validation_sequence_set = [
        [
            BallIsAlwaysOnGround(),
            NeverExcessivelyDribbles(),
        ]
    ]
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
