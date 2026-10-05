import sys

import proto.import_all_protos as protos
import software.python_bindings as tbots_cpp
from proto.ssl_gc_common_pb2 import Team as SslTeam
from software.gameplay_tests.field_test_fixture import WORLD_BUFFER_TIMEOUT
from software.gameplay_tests.simulated_test_fixture import pytest_main
from software.logger.logger import create_logger

logger = create_logger(__name__)


def test_ball_placement(field_test_runner):
    placement_point = tbots_cpp.Point(0, 0)

    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    print("Here are the robots:")
    print(
        [
            robot.current_state.global_position
            for robot in world.friendly_team.team_robots
        ]
    )

    field_test_runner.send_gamecontroller_command(
        protos.Command.Type.BALL_PLACEMENT, SslTeam.BLUE, placement_point
    )

    field_test_runner.run_test(
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )

    # Send a halt tactic after the test finishes
    field_test_runner.set_tactics(
        blue_tactics={
            robot.id: protos.HaltTactic() for robot in world.friendly_team.team_robots
        },
        yellow_tactics=None,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing and -vv increases verbosity
    sys.exit(pytest_main(__file__))
