import software.python_bindings as tbots_cpp
from software.simulated_tests.validation.robot_enters_region import (
    NumberOfRobotsEventuallyExitsRegion,
    NumberOfRobotsEventuallyEntersRegion,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.play_pb2 import PlayName
from proto.import_all_protos import Command
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_example_play(simulated_test_runner):
    ball_initial_pos = tbots_cpp.Point(0, 0)

    def setup(*args):
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
            gc_command=Command.Type.NORMAL_START, team=Team.BLUE
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.DIRECT, team=Team.BLUE
        )

        simulated_test_runner.set_plays(
            blue_play=PlayName.ExamplePlay, yellow_play=PlayName.HaltPlay
        )

    eventually_validations = [
        [
            NumberOfRobotsEventuallyEntersRegion(
                regions=[tbots_cpp.Circle(ball_initial_pos, 1.15)], req_robot_cnt=6
            ),
            NumberOfRobotsEventuallyExitsRegion(
                regions=[tbots_cpp.Circle(ball_initial_pos, 0.9)], req_robot_cnt=6
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
