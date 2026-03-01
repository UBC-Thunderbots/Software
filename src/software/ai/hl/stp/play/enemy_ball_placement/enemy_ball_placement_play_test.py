import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.robot_enters_placement_region import *
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "ball_start_point, ball_placement_point",
    [
        # test normal ball placement
        (tbots_cpp.Point(0, 0), tbots_cpp.Point(-2, 0.5)),
        # test edge case where stadium goes through defense area
        (tbots_cpp.Point(-4.1, 2.8), tbots_cpp.Point(-4.1, -2.8)),
    ],
)
def test_two_ai_ball_placement(
    simulated_test_runner, ball_start_point, ball_placement_point
):
    def setup(*args):
        blue_bots = [
            tbots_cpp.Point(-4.5, 0),
            tbots_cpp.Point(-4, 0.5),
            tbots_cpp.Point(-4, -0.5),
            tbots_cpp.Point(-2, 1),
            tbots_cpp.Point(-2, 0),
            tbots_cpp.Point(-2, -1),
        ]

        yellow_bots = [
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(1, 2.5),
            tbots_cpp.Point(1, -2.5),
            tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
            tbots_cpp.Field.createSSLDivisionBField()
            .enemyDefenseArea()
            .negXNegYCorner(),
            tbots_cpp.Field.createSSLDivisionBField()
            .enemyDefenseArea()
            .negXPosYCorner(),
        ]

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.BALL_PLACEMENT,
            team=Team.YELLOW,
            final_ball_placement_point=ball_placement_point,
        )

        blue_play = Play()
        blue_play.name = PlayName.EnemyBallPlacementPlay

        simulated_test_runner.set_play(blue_play, is_friendly=True)

        yellow_play = Play()
        yellow_play.name = PlayName.BallPlacementPlay

        simulated_test_runner.set_play(yellow_play, is_friendly=False)

        simulated_test_runner.set_world_state(
            create_world_state(
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_start_point,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

    always_validation_sequence_set = [
        [RobotNeverEntersPlacementRegion(ball_placement_point)]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=always_validation_sequence_set,
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
