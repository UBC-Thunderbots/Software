import software.python_bindings as tbots_cpp
import pytest

from proto.import_all_protos import (
    HaltTactic,
)
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.robot_speed_threshold import (
    RobotSpeedEventuallyBelowThreshold,
)
from software.simulated_tests.pytest_validations.delay_validation import DelayValidation
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "blue_robot_locations,blue_robot_velocities",
    [
        (
            [tbots_cpp.Point(-3, 2.5), tbots_cpp.Point(0, 0)],
            [tbots_cpp.Vector(0, 0), tbots_cpp.Vector(0, 0)],
        ),
        (
            [tbots_cpp.Point(-3, 2.5), tbots_cpp.Point(0, 0)],
            [tbots_cpp.Vector(0, 0), tbots_cpp.Vector(4, 4)],
        ),
        (
            [tbots_cpp.Point(-3, 2.5), tbots_cpp.Point(0, -1)],
            [tbots_cpp.Vector(8, 0), tbots_cpp.Vector(4, 4)],
        ),
    ],
)
def test_robot_halt(blue_robot_locations, blue_robot_velocities, simulated_test_runner):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=blue_robot_locations,
                blue_robot_velocities=blue_robot_velocities,
                yellow_robot_locations=[tbots_cpp.Point(4, 0)],
                ball_location=tbots_cpp.Point(0, 0.5),
                ball_velocity=tbots_cpp.Vector(0, 0),
            )
        )

        simulated_test_runner.set_tactics(blue_tactics={1: HaltTactic()})

    robot_stopped_validation = RobotSpeedEventuallyBelowThreshold(speed_threshold=0.001)

    eventually_validations = [
        [
            robot_stopped_validation,
            DelayValidation(
                delay_s=1,
                validation=robot_stopped_validation,
            ),
            DelayValidation(
                delay_s=2,
                validation=robot_stopped_validation,
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
    )


if __name__ == "__main__":
    pytest_main(__file__)
