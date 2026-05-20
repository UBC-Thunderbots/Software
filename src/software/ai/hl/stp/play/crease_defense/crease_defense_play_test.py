import software.python_bindings as tbots_cpp
from proto.play_pb2 import PlayName
from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from software.simulated_tests.validation.robot_speed_threshold import (
    RobotSpeedEventuallyBelowThreshold,
)
from software.simulated_tests.validation.robot_enters_region import (
    NumberOfRobotsEventuallyEntersRegion,
)
from software.simulated_tests.validation.delay_validation import DelayValidation


def test_crease_defense_play(simulated_test_runner):
    field = tbots_cpp.Field.createSSLDivisionBField()
    goalie_position = tbots_cpp.Point(-4.5, 0)

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    goalie_position,
                    tbots_cpp.Point(-3, 1.5),
                    tbots_cpp.Point(-3, 0.5),
                    tbots_cpp.Point(-3, -0.5),
                    tbots_cpp.Point(-3, -1.5),
                    tbots_cpp.Point(-3, -3.0),
                ],
                yellow_robot_locations=[
                    tbots_cpp.Point(1, 3),
                    tbots_cpp.Point(1, -0.25),
                    tbots_cpp.Point(1, -1.25),
                    field.enemyGoalCenter(),
                    field.enemyDefenseArea().negXNegYCorner(),
                    field.enemyDefenseArea().negXPosYCorner(),
                ],
                ball_location=tbots_cpp.Point(0.9, 2.85),
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.set_plays(
            blue_play=PlayName.CreaseDefensePlay, yellow_play=PlayName.HaltPlay
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )

    eventually_validations = [
        [
            # Robots start moving and wait for all robots to come to a halt
            DelayValidation(
                delay_s=1, validation=RobotSpeedEventuallyBelowThreshold(0.001)
            ),
            # Two friendly crease defenders should be close to the goalie
            NumberOfRobotsEventuallyEntersRegion(
                regions=[
                    tbots_cpp.Rectangle(
                        goalie_position + tbots_cpp.Vector(0, -1),
                        goalie_position + tbots_cpp.Vector(1.5, 1),
                    )
                ],
                req_robot_cnt=3,
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
