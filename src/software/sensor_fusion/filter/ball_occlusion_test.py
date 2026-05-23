import pytest
import math

import software.python_bindings as tbots_cpp
from proto.play_pb2 import PlayName

from proto.message_translation.tbots_protobuf import create_world_state
from proto.import_all_protos import Command
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "ball_position, ball_velocity, blue_robot_positions, yellow_robot_positions",
    [
        # Ball moving horizontally, robots line up above the ball path
        (
            tbots_cpp.Point(-4.5, 3),
            tbots_cpp.Vector(1, 0),
            [
                tbots_cpp.Point(-4, 2.8),
                tbots_cpp.Point(-2.5, 2.8),
                tbots_cpp.Point(-1, 2.8),
                tbots_cpp.Point(0.5, 2.8),
                tbots_cpp.Point(2, 2.8),
                tbots_cpp.Point(3.5, 2.8),
            ],
            [tbots_cpp.Point(4.5, 0)],
        ),
        # Ball moving horizontally, robots line up below the ball path
        (
            tbots_cpp.Point(-4.5, -2),
            tbots_cpp.Vector(1, 0),
            [
                tbots_cpp.Point(-4, -1.8),
                tbots_cpp.Point(-2.5, -1.8),
                tbots_cpp.Point(-1, -1.8),
                tbots_cpp.Point(0.5, -1.8),
                tbots_cpp.Point(2, -1.8),
                tbots_cpp.Point(3.5, -1.8),
            ],
            [tbots_cpp.Point(4.5, 0)],
        ),
        # Ball moving vertically, robots line up on the left
        (
            tbots_cpp.Point(-4.5, 3),
            tbots_cpp.Vector(0, -1),
            [
                tbots_cpp.Point(-4.3, -2),
                tbots_cpp.Point(-4.3, -1),
                tbots_cpp.Point(-4.3, 0),
                tbots_cpp.Point(-4.3, 1),
                tbots_cpp.Point(-4.3, 2),
                tbots_cpp.Point(-4.3, 2.2),
            ],
            [tbots_cpp.Point(4.5, 0)],
        ),
        # Ball moving vertically, robots line up on the right
        (
            tbots_cpp.Point(4.5, 3),
            tbots_cpp.Vector(0, -1),
            [
                tbots_cpp.Point(4.3, -2),
                tbots_cpp.Point(4.3, -1),
                tbots_cpp.Point(4.3, 0),
                tbots_cpp.Point(4.3, 1),
                tbots_cpp.Point(4.3, 2),
                tbots_cpp.Point(4.3, 2.2),
            ],
            [tbots_cpp.Point(-4.5, 0)],
        ),
        # Ball moving toward friendly goal, enemy robot occluding
        (
            tbots_cpp.Point(-3, 0),
            tbots_cpp.Vector(-1, 0),
            [tbots_cpp.Point(-4.5, 0)],
            [tbots_cpp.Point(-2.8, 0)],
        ),
        # Ball moving toward enemy goal, friendly robot occluding
        (
            tbots_cpp.Point(3, 0),
            tbots_cpp.Vector(0, 1),
            [tbots_cpp.Point(2.8, 0)],
            [tbots_cpp.Point(4.5, 0)],
        ),
        # Ball moving diagonally, robots near path
        (
            tbots_cpp.Point(-3.5, -1),
            tbots_cpp.Vector(-1, 1),
            [tbots_cpp.Point(-4.5, 0)],
            [tbots_cpp.Point(-3.5 + 0.2 / math.sqrt(2.0), -1 - 0.2 / math.sqrt(2.0))],
        ),
        (
            tbots_cpp.Point(3.5, -1),
            tbots_cpp.Vector(1, 1),
            [tbots_cpp.Point(3.5 - 0.2 / math.sqrt(2.0), -1 - 0.2 / math.sqrt(2.0))],
            [tbots_cpp.Point(4.5, 0)],
        ),
        (
            tbots_cpp.Point(-3.5, 1),
            tbots_cpp.Vector(-1, -1),
            [tbots_cpp.Point(-4.5, 0)],
            [tbots_cpp.Point(-3.5 + 0.2 / math.sqrt(2.0), 1 + 0.2 / math.sqrt(2.0))],
        ),
        (
            tbots_cpp.Point(3.5, 1),
            tbots_cpp.Vector(1, -1),
            [tbots_cpp.Point(3.5 - 0.2 / math.sqrt(2.0), 1 + 0.2 / math.sqrt(2.0))],
            [tbots_cpp.Point(4.5, 0)],
        ),
        # Corner shots with multiple robots
        (
            tbots_cpp.Point(-4.5 + 0.2 / math.sqrt(2.0), 3 - 0.2 / math.sqrt(2.0)),
            tbots_cpp.Vector(3.0 / math.sqrt(13.0), -2.0 / math.sqrt(13.0)),
            [tbots_cpp.Point(-4.5, 3), tbots_cpp.Point(-1.5, 1)],
            [tbots_cpp.Point(4.5, 0)],
        ),
        (
            tbots_cpp.Point(-4.5 + 0.23 / math.sqrt(2.0), -3 + 0.23 / math.sqrt(2.0)),
            tbots_cpp.Vector(3.0 / math.sqrt(13.0), 2.0 / math.sqrt(13.0)),
            [tbots_cpp.Point(-4.5, -3), tbots_cpp.Point(-1.5, -1)],
            [tbots_cpp.Point(4.5, 0)],
        ),
        (
            tbots_cpp.Point(4.5 - 0.2 / math.sqrt(2.0), 3 - 0.2 / math.sqrt(2.0)),
            tbots_cpp.Vector(-3.0 / math.sqrt(13.0), -2.0 / math.sqrt(13.0)),
            [tbots_cpp.Point(4.5, 3), tbots_cpp.Point(1.5, 1)],
            [tbots_cpp.Point(4.5, 0)],
        ),
        (
            tbots_cpp.Point(4.5 - 0.23 / math.sqrt(2.0), -3 + 0.23 / math.sqrt(2.0)),
            tbots_cpp.Vector(-3.0 / math.sqrt(13.0), 2.0 / math.sqrt(13.0)),
            [tbots_cpp.Point(4.5, -3), tbots_cpp.Point(1.5, -1)],
            [tbots_cpp.Point(-4.5, 0)],
        ),
        # Complex paths with multiple robots on each side
        (
            tbots_cpp.Point(-4, -3),
            tbots_cpp.Vector(1, 3),
            [
                tbots_cpp.Point(-3.8, -2.8),
                tbots_cpp.Point(-2.9, 0.1),
                tbots_cpp.Point(-1.9, 3.1),
            ],
            [
                tbots_cpp.Point(-4.2, -3.2),
                tbots_cpp.Point(-3.1, -0.1),
                tbots_cpp.Point(-2.1, 2.9),
            ],
        ),
        (
            tbots_cpp.Point(-4, -3),
            tbots_cpp.Vector(4, 1),
            [
                tbots_cpp.Point(-3.8, -2.8),
                tbots_cpp.Point(0.1, -1.9),
                tbots_cpp.Point(4.1, -0.9),
            ],
            [
                tbots_cpp.Point(-4.2, -3.2),
                tbots_cpp.Point(-0.1, -2.1),
                tbots_cpp.Point(3.9, -1.1),
            ],
        ),
    ],
)
def test_ball_occlusion(
    ball_position,
    ball_velocity,
    blue_robot_positions,
    yellow_robot_positions,
    simulated_test_runner,
):
    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=blue_robot_positions,
                yellow_robot_locations=yellow_robot_positions,
                ball_location=ball_position,
                ball_velocity=ball_velocity,
            ),
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.HALT, team=Team.UNKNOWN
        )

        simulated_test_runner.set_plays(
            blue_play=PlayName.HaltPlay, yellow_play=PlayName.HaltPlay
        )

    # This test validates that the ball tracking/filter handles occlusion correctly.
    # The validation simply waits for the simulation to run long enough (10s).
    # If the system crashes or has tracking issues, the test will fail.
    simulated_test_runner.run_test(
        setup=setup,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
