import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_crease_defense_play(simulated_test_runner):
    def setup(*args):
        ball_initial_pos = tbots_cpp.Point(0.9, 2.85)

        blue_bots = [
            tbots_cpp.Point(-4.5, 0),
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(-3, 0.5),
            tbots_cpp.Point(-3, -0.5),
            tbots_cpp.Point(-3, -1.5),
            tbots_cpp.Point(-3, -3.0),
        ]

        yellow_bots = [
            tbots_cpp.Point(1, 3),
            tbots_cpp.Point(1, -0.25),
            tbots_cpp.Point(1, -1.25),
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
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

        blue_play = Play()
        blue_play.name = PlayName.CreaseDefensePlay

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay

        simulated_test_runner.set_play(blue_play, is_friendly=True)
        simulated_test_runner.set_play(yellow_play, is_friendly=False)

        simulated_test_runner.set_world_state(
            create_world_state(
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

    always_validation_sequence_set = [[]]

    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=25,
    )


if __name__ == "__main__":
    pytest_main(__file__)
