import proto.import_all_protos as protos
import software.python_bindings as tbots_cpp
from software.gameplay_tests.util import pytest_main
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.gameplay_tests.validation.ball_enters_region import (
    BallAlwaysStaysInRegion,
)
from software.gameplay_tests.validation.excessive_dribbling import (
    NeverExcessivelyDribbles,
)
from software.gameplay_tests.validation.friendly_has_ball_possession import (
    FriendlyAlwaysHasBallPossession,
)
from software.gameplay_tests.validation.friendly_team_scored import (
    FriendlyTeamEventuallyScored,
)


def test_offense_play(gameplay_test_runner):
    start_point = tbots_cpp.Point(-4.4, 2.9)

    def setup():
        blue_bots = [
            tbots_cpp.Point(-4.5, 3.0),
            tbots_cpp.Point(-2, 1.5),
            tbots_cpp.Point(-2, 0.5),
            tbots_cpp.Point(-2, -1.7),
            tbots_cpp.Point(-2, -1.5),
            tbots_cpp.Point(-2, -0.5),
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

        gameplay_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=blue_bots,
                yellow_robot_locations=yellow_bots,
                ball_location=start_point,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        gameplay_test_runner.send_gamecontroller_command(
            gc_command=protos.Command.Type.STOP, team=SslTeam.UNKNOWN
        )
        gameplay_test_runner.send_gamecontroller_command(
            gc_command=protos.Command.Type.FORCE_START, team=SslTeam.BLUE
        )

        gameplay_test_runner.set_plays(
            blue_play=protos.PlayName.OffensePlay, yellow_play=protos.PlayName.HaltPlay
        )

    field = tbots_cpp.Field.createSSLDivisionBField()

    # Always Validation
    always_validation_sequence_set = [
        [BallAlwaysStaysInRegion(regions=[field.fieldBoundary()])],
        [NeverExcessivelyDribbles()],
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [FriendlyTeamEventuallyScored()],
    ]

    gameplay_test_runner.run_test(
        setup=setup,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
