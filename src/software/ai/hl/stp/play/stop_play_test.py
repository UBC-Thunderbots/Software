import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from proto.play_pb2 import Play, PlayName
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.robot_speed_threshold import (
    RobotSpeedEventuallyBelowThreshold,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_stop_play(simulated_test_runner):
    """Test stop play: robots slow down and avoid the ball (STOP referee state)."""

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

        # Game controller: STOP (both teams) - stop play behaviour
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.FORCE_START, team=Team.UNKNOWN
        )

        # Force play override: blue runs StopPlay, yellow runs HaltPlay
        blue_play = Play()
        blue_play.name = PlayName.StopPlay

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            Play, yellow_play
        )

        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState,
            create_world_state(
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

    # After ~8s the C++ test expects robots to be slow; allow 10s for test
    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[
            [RobotSpeedEventuallyBelowThreshold(1.5)]
        ],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
