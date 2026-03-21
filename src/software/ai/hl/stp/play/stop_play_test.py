import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName

from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.robot_speed_threshold import (
    RobotSpeedEventuallyBelowThreshold,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_stop_play(simulated_test_runner):
    def setup(*args):
        # Ball at centre (matches C++ test_stop_play_ball_at_centre_robots_spread_out)
        ball_initial_pos = tbots_cpp.Point(0, 0)

        field = tbots_cpp.Field.createSSLDivisionBField()

        blue_bots = [
            tbots_cpp.Point(-4, 0),
            tbots_cpp.Point(-0.3, 0),
            tbots_cpp.Point(0.3, 0),
            tbots_cpp.Point(0, 0.3),
            tbots_cpp.Point(-3, -1.5),
            tbots_cpp.Point(4.6, -3.1),
        ]

        yellow_bots = [
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(1, 2.5),
            tbots_cpp.Point(1, -2.5),
            field.enemyGoalCenter(),
            field.enemyDefenseArea().negXNegYCorner(),
            field.enemyDefenseArea().negXPosYCorner(),
        ]

        simulated_test_runner.set_world_state(
            create_world_state(
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        # Game controller: STOP (both teams) - stop play behaviour
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.FORCE_START, team=Team.UNKNOWN
        )

        blue_play = Play()
        blue_play.name = PlayName.StopPlay

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay

        simulated_test_runner.set_play(blue_play, is_friendly=True)
        simulated_test_runner.set_play(yellow_play, is_friendly=False)

    # C++ test waits 8s before checking; use 15s timeout so robots have time to slow.
    # Threshold 1.4 m/s: expect robots to eventually slow below the 1.5 m/s STOP limit.
    # TODO (#3638): add an eventually-validation that friendly robots stay at least 0.5 m away
    # from the ball once pytest fixtures support delaying validations (to mirror the
    # C++ robotsAvoidBall(0.5, ...) check).
    eventually_validations = [[RobotSpeedEventuallyBelowThreshold(1.4)]]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
