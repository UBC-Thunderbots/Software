import pytest

import software.python_bindings as tbots_cpp
from software.py_constants import ROBOT_MAX_RADIUS_METERS
from proto.play_pb2 import PlayName

from software.simulated_tests.validation.robot_enters_region import (
    NumberOfRobotsEventuallyEntersRegion,
)
from software.simulated_tests.validation.robot_at_position import (
    RobotEventuallyAtPosition,
)
from software.simulated_tests.validation.robot_at_orientation import (
    RobotEventuallyAtOrientation,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.import_all_protos import Command
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "friendly_robot_positions, enemy_distance_behind_ball",
    [
        # Test 1: robots scattered, enemy 1m behind ball
        (
            [
                tbots_cpp.Point(1, 2),
                tbots_cpp.Point(-1, -2),
                tbots_cpp.Point(-2.5, 3),
                tbots_cpp.Point(2, -1),
                tbots_cpp.Point(0, 3),
                tbots_cpp.Point(3, 0),
            ],
            1.0,
        ),
        # Test 2-5: same robot positions, varying distances
        (
            [
                tbots_cpp.Point(2.2, 1.2),
                tbots_cpp.Point(-0.5, -2.1),
                tbots_cpp.Point(-2.5, 1.3),
                tbots_cpp.Point(1.2, -1.5),
                tbots_cpp.Point(0, 2),
                tbots_cpp.Point(1, 0),
            ],
            1.3,
        ),
        (
            [
                tbots_cpp.Point(2.2, 1.2),
                tbots_cpp.Point(-0.5, -2.1),
                tbots_cpp.Point(-2.5, 1.3),
                tbots_cpp.Point(1.2, -1.5),
                tbots_cpp.Point(0, 2),
                tbots_cpp.Point(1, 0),
            ],
            1.4,
        ),
        (
            [
                tbots_cpp.Point(2.2, 1.2),
                tbots_cpp.Point(-0.5, -2.1),
                tbots_cpp.Point(-2.5, 1.3),
                tbots_cpp.Point(1.2, -1.5),
                tbots_cpp.Point(0, 2),
                tbots_cpp.Point(1, 0),
            ],
            1.45,
        ),
        (
            [
                tbots_cpp.Point(2.2, 1.2),
                tbots_cpp.Point(-0.5, -2.1),
                tbots_cpp.Point(-2.5, 1.3),
                tbots_cpp.Point(1.2, -1.5),
                tbots_cpp.Point(0, 2),
                tbots_cpp.Point(1, 0),
            ],
            1.5,
        ),
        (
            [
                tbots_cpp.Point(2.2, 1.2),
                tbots_cpp.Point(-0.5, -2.1),
                tbots_cpp.Point(-2.5, 1.3),
                tbots_cpp.Point(1.2, -1.5),
                tbots_cpp.Point(0, 2),
                tbots_cpp.Point(1, 0),
            ],
            1.6,
        ),
    ],
)
def test_penalty_kick_enemy_play_setup(
    friendly_robot_positions,
    enemy_distance_behind_ball,
    simulated_test_runner,
):
    field = tbots_cpp.Field.createSSLDivisionBField()
    ball_initial_pos = field.enemyPenaltyMark()
    enemy_penalty_x = ball_initial_pos.x()

    def setup(*args):
        # Enemy robots behind the penalty mark
        yellow_bots = [
            tbots_cpp.Point(enemy_penalty_x + 0.3, 0),  # kicker robot
            tbots_cpp.Point(enemy_penalty_x + enemy_distance_behind_ball, 0),
            tbots_cpp.Point(
                enemy_penalty_x + enemy_distance_behind_ball,
                8 * ROBOT_MAX_RADIUS_METERS,
            ),
            tbots_cpp.Point(
                enemy_penalty_x + enemy_distance_behind_ball,
                16 * ROBOT_MAX_RADIUS_METERS,
            ),
            tbots_cpp.Point(
                enemy_penalty_x + enemy_distance_behind_ball,
                -8 * ROBOT_MAX_RADIUS_METERS,
            ),
            tbots_cpp.Point(
                enemy_penalty_x + enemy_distance_behind_ball,
                -16 * ROBOT_MAX_RADIUS_METERS,
            ),
        ]

        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=friendly_robot_positions,
                yellow_robot_locations=yellow_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.HALT, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.PENALTY, team=Team.YELLOW
        )

        simulated_test_runner.set_plays(
            blue_play=PlayName.PenaltyKickEnemyPlay,
            yellow_play=PlayName.HaltPlay,
        )

    # Rectangle behind the ball where 5 non-goalie robots should be positioned
    behind_ball_region = tbots_cpp.Rectangle(
        tbots_cpp.Point(enemy_penalty_x + 1, field.enemyCornerNeg().y()),
        field.enemyCornerPos(),
    )

    eventually_validation_sequence_set = [
        [
            # Goalie robot at friendly goal center facing forward
            RobotEventuallyAtOrientation(
                robot_id=0,
                orientation=tbots_cpp.Angle.zero(),
            ),
            RobotEventuallyAtPosition(
                robot_id=0,
                position=field.friendlyGoalCenter(),
            ),
            # 5 other robots in the behind_ball_region
            NumberOfRobotsEventuallyEntersRegion(
                regions=[behind_ball_region], req_robot_cnt=5
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        ag_eventually_validation_sequence_set=eventually_validation_sequence_set,
        test_timeout_s=20,
    )


if __name__ == "__main__":
    pytest_main(__file__)
