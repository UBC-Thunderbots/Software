import proto.import_all_protos as protos
import pytest
import software.python_bindings as tbots_cpp
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.gameplay_tests.util import pytest_main
from software.gameplay_tests.validation.ball_enters_region import (
    BallAlwaysStaysInRegion,
)


@pytest.mark.parametrize(
    "ball_start_point, ball_placement_point",
    [
        # test normal ball placement (not edge case)
        (tbots_cpp.Point(2, 2), tbots_cpp.Point(0, 1.5)),
    ],
)
def test_two_ai_ball_placement(
    gameplay_test_runner, ball_start_point, ball_placement_point
):
    gameplay_test_runner.run_test(
        setup=lambda: ball_placement_play_setup(
            ball_start_point,
            ball_placement_point,
            gameplay_test_runner,
            False,
        ),
        test_timeout_s=5,
        always_validation_sequence_set=[
            [
                BallAlwaysStaysInRegion(
                    regions=[tbots_cpp.Circle(tbots_cpp.Point(0, 0), 50.0)]
                )
            ]
        ],
        gc_cmd_with_delay=[
            (0, protos.Command.Type.STOP, SslTeam.UNKNOWN, None),
            (0, protos.Command.Type.BALL_PLACEMENT, SslTeam.BLUE, ball_placement_point),
        ],
    )


def ball_placement_play_setup(
    ball_start_point, ball_placement_point, gameplay_test_runner, blue_only
):
    # Setup blue robots
    blue_bots = [
        tbots_cpp.Point(-2.75, 1.5),
        tbots_cpp.Point(-0.0, 0.0),
        tbots_cpp.Point(-2.75, -0.5),
        tbots_cpp.Field.createSSLDivisionBField().friendlyGoalCenter(),
        tbots_cpp.Field.createSSLDivisionBField()
        .friendlyDefenseArea()
        .negXNegYCorner(),
        tbots_cpp.Field.createSSLDivisionBField()
        .friendlyDefenseArea()
        .negXPosYCorner(),
    ]

    yellow_bots = []

    # Optionally skip yellow robots entirely
    if not blue_only:
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

    # Create world state
    gameplay_test_runner.set_world_state(
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_start_point,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    # Force play override here
    gameplay_test_runner.set_plays(
        blue_play=protos.PlayName.BallPlacementPlay,
        yellow_play=protos.PlayName.HaltPlay,
    )


if __name__ == "__main__":
    pytest_main(__file__)
