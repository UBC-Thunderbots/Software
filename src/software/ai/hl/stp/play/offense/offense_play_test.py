import software.python_bindings as tbots_cpp
from proto.play_pb2 import PlayName
from software.gameplay_tests.validation.friendly_team_scored import *
from software.gameplay_tests.validation.ball_enters_region import *
from software.gameplay_tests.validation.excessive_dribbling import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.gameplay_tests.util import pytest_main


def test_offense_play(simulated_test_runner):
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

        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=blue_bots,
                yellow_robot_locations=yellow_bots,
                ball_location=start_point,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

        simulated_test_runner.set_plays(
            blue_play=PlayName.OffensePlay, yellow_play=PlayName.HaltPlay
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

    simulated_test_runner.run_test(
        setup=setup,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
