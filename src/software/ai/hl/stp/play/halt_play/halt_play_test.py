import software.python_bindings as tbots_cpp
from software.gameplay_tests.validation.robot_speed_threshold import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.gameplay_tests.util import pytest_main


# TODO issue  #2599 - Remove Duration parameter from test
# @pytest.mark.parametrize("run_enemy_ai,test_duration", [(False, 20), (True, 20)])
def test_halt_play(gameplay_test_runner):
    def setup():
        ball_initial_pos = tbots_cpp.Point(0, 0)

        blue_bots = [
            tbots_cpp.Point(-3, 2.5),
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(-3, 0.5),
            tbots_cpp.Point(-3, -0.5),
            tbots_cpp.Point(-3, -1.5),
            tbots_cpp.Point(-3, -2.5),
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
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        gameplay_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        gameplay_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.FORCE_START, team=Team.UNKNOWN
        )

    gameplay_test_runner.run_test(
        setup=setup,
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[
            [RobotSpeedEventuallyBelowThreshold(1e-3)]
        ],
        ci_cmd_with_delay=[
            (3, Command.Type.HALT, Team.BLUE),
            (3, Command.Type.HALT, Team.YELLOW),
        ],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
