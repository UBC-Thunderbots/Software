import software.python_bindings as tbots_cpp

from proto.import_all_protos import (
    HaltTactic,
)
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.robot_speed_threshold import (
    RobotSpeedEventuallyBelowThreshold,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def test_robot_already_stopped(simulated_test_runner):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[tbots_cpp.Point(-3, 2.5), tbots_cpp.Point(0, 0)],
                yellow_robot_locations=[tbots_cpp.Point(4, 0)],
                ball_location=tbots_cpp.Point(0, 0.5),
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(blue_tactics={1: HaltTactic()})

    eventually_validations = [
        [
            # TODO: check for 1000 ticks
            RobotSpeedEventuallyBelowThreshold(speed_threshold=0.001),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        inv_always_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
    )


def test_robot_start_moving(simulated_test_runner):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[tbots_cpp.Point(-3, 2.5), tbots_cpp.Point(0, 0)],
                blue_robot_velocities=[tbots_cpp.Vector(0, 0), tbots_cpp.Vector(4, 4)],
                yellow_robot_locations=[tbots_cpp.Point(4, 0)],
                ball_location=tbots_cpp.Point(0, 0.5),
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(blue_tactics={1: HaltTactic()})

    eventually_validations = [
        [
            # TODO: check for 1000 ticks
            RobotSpeedEventuallyBelowThreshold(speed_threshold=0.001),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        inv_always_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
    )


if __name__ == "__main__":
    pytest_main(__file__)
