import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.friendly_team_scored import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.friendly_has_ball_possession import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_offense_play(simulated_test_runner):
    def setup(start_point):
        ball_initial_pos = start_point

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
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

        blue_play = Play()
        blue_play.name = PlayName.OffensePlay

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay

        simulated_test_runner.set_play(blue_play, is_friendly=True)
        simulated_test_runner.set_play(yellow_play, is_friendly=False)

    field = tbots_cpp.Field.createSSLDivisionBField()

    # Always Validation
    inv_always_validation_sequence_set = [
        [BallAlwaysStaysInRegion(regions=[field.fieldBoundary()])]
    ]

    ag_always_validation_sequence_set = [[FriendlyAlwaysHasBallPossession()]]

    # Eventually Validation
    inv_eventually_validation_sequence_set = [[]]
    ag_eventually_validation_sequence_set = [[FriendlyTeamEventuallyScored()]]

    simulated_test_runner.run_test(
        params=[tbots_cpp.Point(-4.4, 2.9)],
        setup=setup,
        inv_eventually_validation_sequence_set=inv_eventually_validation_sequence_set,
        inv_always_validation_sequence_set=inv_always_validation_sequence_set,
        ag_eventually_validation_sequence_set=ag_eventually_validation_sequence_set,
        ag_always_validation_sequence_set=ag_always_validation_sequence_set,
        test_timeout_s=15,
        run_till_end=True,
    )


if __name__ == "__main__":
    pytest_main(__file__)
