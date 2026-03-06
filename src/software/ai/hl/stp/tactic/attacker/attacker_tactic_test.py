import pytest
import software.python_bindings as tbots_cpp
from software.py_constants import ROBOT_MAX_RADIUS_METERS

from proto.import_all_protos import AttackerTactic, Pass
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.ball_kicked_in_direction import (
    BallEventuallyKickedInDirection,
)
from software.simulated_tests.pytest_validations.excessive_dribbling import (
    NeverExcessivelyDribbles,
)
from software.simulated_tests.pytest_validations.friendly_team_scored import (
    FriendlyTeamEventuallyScored,
)
from software.simulated_tests.pytest_validations.robot_at_orientation import (
    RobotEventuallyAtOrientation,
)
from software.simulated_tests.pytest_validations.robot_at_position import (
    RobotEventuallyAtPosition,
)
from software.simulated_tests.simulated_test_fixture import pytest_main


def calculate_ball_velocity(passer_point, receiver_point, speed):
    """Calculate ball velocity based on pass trajectory"""
    direction = receiver_point - passer_point
    if direction.length() > 0:
        direction = direction.normalize()
    return direction * speed


# Passing Tests
@pytest.mark.parametrize(
    "passer_point, receiver_point, pass_speed, robot_pos, ball_pos, ball_velocity",
    [
        # Stationary Ball Tests
        # Attacker point != Balls location & Balls location != Robots Location
        (
            tbots_cpp.Point(0, 0.5),
            tbots_cpp.Point(0, 0),
            5,
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0.5, 0.5),
            tbots_cpp.Vector(0, 0),
        ),
        # Attacker point == Balls location & Balls location != Robots Location
        (
            tbots_cpp.Point(-0.5, -0.5),
            tbots_cpp.Point(0, 0),
            5,
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(-0.5, -0.5),
            tbots_cpp.Vector(0, 0),
        ),
        # Attacker point != Balls location & Balls location == Robots Location
        (
            tbots_cpp.Point(0.5, 0.5),
            tbots_cpp.Point(0, 1),
            5,
            tbots_cpp.Point(0.5, 0.5),
            tbots_cpp.Point(-0.5, 0.5),
            tbots_cpp.Vector(0, 0),
        ),
        # TODO(#2909): Enable test once the robot can turn faster and hits the ball with
        # the ball at the exact robot location
        # Attacker point == Balls location & Balls location == Robots Location
        # (
        #     tbots_cpp.Point(0.0, 0.0),
        #     tbots_cpp.Point(0, 0),
        #     5,
        #     tbots_cpp.Point(0, 0),
        #     tbots_cpp.Point(0.0, 0.0),
        #     tbots_cpp.Vector(0, 0),
        # ),
        # Attacker point far away (not a normal use case, but just to sanity check)
        (
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0, 0),
            5,
            tbots_cpp.Point(3.5, 2.5),
            tbots_cpp.Point(0.0, 0.0),
            tbots_cpp.Vector(0, 0),
        ),
        # Attacker point != Balls location & Balls location != Robots Location (duplicate)
        (
            tbots_cpp.Point(0, 0.5),
            tbots_cpp.Point(0, 0),
            5,
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0.5, 0.5),
            tbots_cpp.Vector(0, 0),
        ),
        # Moving Ball Tests
        # Attacker point == Balls location & Balls location != Robots Location
        (
            tbots_cpp.Point(-0.5, -0.5),
            tbots_cpp.Point(0, 0),
            5,
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(-0.5, -0.5),
            tbots_cpp.Vector(1, 0),
        ),
        # TODO (#2859): Robot does not kick ball when dribbler is off since it is
        # too far away.
        # Attacker point != Balls location & Balls location == Robots Location
        # (
        #     tbots_cpp.Point(0.4, 0.4),
        #     tbots_cpp.Point(0, 1),
        #     5,
        #     tbots_cpp.Point(0.5, 0.5),
        #     tbots_cpp.Point(-0.4, 0.4),
        #     tbots_cpp.Vector(0, 1),
        # ),
        # Attacker point == Balls location & Balls location == Robots Location
        (
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(0, 0),
            5,
            tbots_cpp.Point(0, 0),
            tbots_cpp.Point(ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS),
            tbots_cpp.Vector(1, 0),
        ),
    ],
)
def test_attacker_passing(
    passer_point,
    receiver_point,
    pass_speed,
    robot_pos,
    ball_pos,
    ball_velocity,
    simulated_test_runner,
):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    robot_pos,
                ],
                # TODO (#2558)
                # In C++ test: Uses ai_config to set min_open_angle_for_shot_deg to force passing
                # Temp fix by blocking off the enemy goal
                yellow_robot_locations=[
                    tbots_cpp.Point(4, -0.4),
                    tbots_cpp.Point(4, -0.2),
                    tbots_cpp.Point(4, 0),
                    tbots_cpp.Point(4, 0.2),
                    tbots_cpp.Point(4, 0.4),
                ],
                ball_location=ball_pos,
                ball_velocity=ball_velocity,
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: AttackerTactic(
                    best_pass_so_far=Pass(
                        passer_point=tbots_cpp.createPointProto(passer_point),
                        receiver_point=tbots_cpp.createPointProto(receiver_point),
                        pass_speed_m_per_s=pass_speed,
                    ),
                    pass_committed=True,
                )
            }
        )

    pass_orientation = (receiver_point - passer_point).orientation()

    eventually_validation_sequence_set = [
        [
            RobotEventuallyAtOrientation(robot_id=1, orientation=pass_orientation),
            # Larger threshold since techically ball is at passer point, not the robot
            RobotEventuallyAtPosition(
                robot_id=1, position=passer_point, threshold=ROBOT_MAX_RADIUS_METERS
            ),
            BallEventuallyKickedInDirection(kick_direction=pass_orientation),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        run_till_end=False,
        test_timeout_s=7,
    )


# Keep Away Tests
@pytest.mark.parametrize(
    "passer_point, receiver_point, pass_speed, robot_pos, robot_orientation, ball_pos, ball_velocity, enemy_positions, ignore_score_checks",
    [
        # Basic keep away with enemy robots around
        (
            tbots_cpp.Point(-0.2, 0.0),
            tbots_cpp.Point(-3, 2.5),
            5,
            tbots_cpp.Point(0.25, 0),
            tbots_cpp.Angle.half(),
            tbots_cpp.Point(0.0, 0.0),
            tbots_cpp.Vector(0, 0),
            [
                tbots_cpp.Point(-0.6, 0.25),
                tbots_cpp.Point(0.0, 0.6),
                tbots_cpp.Point(-0.25, 0.5),
                tbots_cpp.Point(0.6, -0.25),
            ],
            True,  # This test is too unstable, so we ignore the checks
        ),
        # TODO(#2909): Enable test once the robot can turn faster and hits the ball with
        # the ball at the exact robot location
        # (
        #     tbots_cpp.Point(0.0, 0.0),
        #     tbots_cpp.Point(-3, 2.5),
        #     5,
        #     tbots_cpp.Point(0.25, 0),
        #     tbots_cpp.Angle.zero(),
        #     tbots_cpp.Point(0.0, 0.0),
        #     tbots_cpp.Vector(0, 0),
        #     [tbots_cpp.Point(-0.5, 0.5)],
        #     False,
        # ),
    ],
)
def test_attacker_keep_away(
    passer_point,
    receiver_point,
    pass_speed,
    robot_pos,
    robot_orientation,
    ball_pos,
    ball_velocity,
    enemy_positions,
    ignore_score_checks,
    simulated_test_runner,
):
    # TODO(#2558): Port C++ validation functions that don't exist in Python yet
    # In C++ test:
    # - Uses ai_config to set min_open_angle_for_shot_deg to 90 and
    #   enemy_about_to_steal_ball_radius to 0.01
    # - Validates calculateProximityRisk improves over time
    # - Validates ratePassEnemyRisk improves over time
    # - Validates ball stays in field boundaries
    # This test is skipped in C++ due to instability

    field = tbots_cpp.Field.createSSLDivisionBField()
    field_top_left = field.fieldLines().negXPosYCorner()

    # Keep away near field corner (second test case from C++)
    def setup(*args):
        robot_pos = field_top_left
        ball_pos = tbots_cpp.Point(field_top_left.x() + 0.05, field_top_left.y() - 0.2)
        enemy_positions = [
            tbots_cpp.Point(-4, 2),
            tbots_cpp.Point(-4, 2.25),
            tbots_cpp.Point(-4, 2.5),
            tbots_cpp.Point(-4, 2.75),
        ]

        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    tbots_cpp.Point(-3, 2.5),
                    robot_pos,
                ],
                yellow_robot_locations=enemy_positions,
                ball_location=ball_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        pass_point = tbots_cpp.Point(
            field_top_left.x() + 0.05, field_top_left.y() - 0.05
        )
        receiver_point = tbots_cpp.Point(0, 0)

        simulated_test_runner.set_tactics(
            blue_tactics={
                1: AttackerTactic(
                    best_pass_so_far=Pass(
                        passer_point=tbots_cpp.createPointProto(pass_point),
                        receiver_point=tbots_cpp.createPointProto(receiver_point),
                        pass_speed_m_per_s=5,
                    ),
                    pass_committed=False,
                )
            }
        )

    always_validation_sequence_set = [
        [
            NeverExcessivelyDribbles(),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_always_validation_sequence_set=always_validation_sequence_set,
    )


# Shoot Goal Tests
@pytest.mark.parametrize(
    "ball_pos, ball_velocity, robot_pos, enemy_positions",
    [
        # # TODO (#2693): Fix FSM looping endlessly
        # # enemy goal blocked by enemy robots with enemy threat right
        # (
        #     tbots_cpp.Point(2, 1),
        #     tbots_cpp.Vector(0, 0),
        #     tbots_cpp.Point(1, 1),
        #     [
        #         tbots_cpp.Point(2.4, 1),
        #         tbots_cpp.Point(3, 0.4),
        #         tbots_cpp.Point(3, 0.8),
        #         tbots_cpp.Point(3.1, 0.6),
        #         tbots_cpp.Point(3.1, 1),
        #         tbots_cpp.Point(4.2, 1.2),
        #     ],
        # ),
        # small opening in enemy formation
        (
            tbots_cpp.Point(2, 1),
            tbots_cpp.Vector(0, 0),
            tbots_cpp.Point(1, 1),
            [
                tbots_cpp.Point(1, 0),
                tbots_cpp.Point(3, 0.2),
                tbots_cpp.Point(3, 0.8),
                tbots_cpp.Point(3.1, 0),
                tbots_cpp.Point(3.1, 1),
                tbots_cpp.Point(4.2, 1.2),
            ],
        ),
        # extreme angle shot
        (
            tbots_cpp.Point(3.8, -1.9),
            tbots_cpp.Vector(0, 0),
            tbots_cpp.Point(1, 1),
            [
                tbots_cpp.Point(1, 0),
                tbots_cpp.Point(3, 1.2),
                tbots_cpp.Point(3, 0.8),
                tbots_cpp.Point(3.1, 0.6),
                tbots_cpp.Point(3.1, 1),
                tbots_cpp.Point(4.2, 0.5),
            ],
        ),
        # enemy trying to steal
        (
            tbots_cpp.Point(2.5, -1),
            tbots_cpp.Vector(0, 0),
            tbots_cpp.Point(1, 1),
            [
                tbots_cpp.Point(2.5, -1.4),
                tbots_cpp.Point(3, 0.4),
                tbots_cpp.Point(3, 0.8),
                tbots_cpp.Point(3.1, 0.6),
                tbots_cpp.Point(3.1, 1),
                tbots_cpp.Point(4.2, 1.2),
            ],
        ),
    ],
)
def test_attacker_shoot_goal(
    ball_pos, ball_velocity, robot_pos, enemy_positions, simulated_test_runner
):
    # TODO (#2558)
    # In C++ test:
    # - Uses ai_config to set pass_delay_sec to 0.0
    # - Uses motion constraint FRIENDLY_DEFENSE_AREA

    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[robot_pos],
                yellow_robot_locations=enemy_positions,
                ball_location=ball_pos,
                ball_velocity=ball_velocity,
            )
        )

        simulated_test_runner.set_tactics(
            blue_tactics={
                0: AttackerTactic(
                    chip_target=tbots_cpp.createPointProto(
                        tbots_cpp.Point(0, field.fieldLines().yMin())
                    ),
                )
            }
        )

    eventually_validation_sequence_set = [
        [
            FriendlyTeamEventuallyScored(),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        run_till_end=False,
        test_timeout_s=9,
    )


if __name__ == "__main__":
    pytest_main(__file__)
