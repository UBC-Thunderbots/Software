import pytest

import software.python_bindings as tbots_cpp
import sys
from proto.ssl_gc_common_pb2 import Team
from proto.ssl_gc_state_pb2 import Command
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *

from software.simulated_tests.simulated_test_fixture import *
from software.logger.logger import createLogger
from software.simulated_tests.robot_enters_region import RobotEventuallyEntersRegion
from proto.message_translation.tbots_protobuf import create_world_state

logger = createLogger(__name__)


def test_ball_placement(field_test_runner):
    # robot ID
    id = 2
    # point to place ball
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
        Command.Type.BALL_PLACEMENT, proto.ssl_gc_common_pb2.Team.BLUE, placement_point
    )
    field_test_runner.run_test(
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )
    # Send a stop tactic after the test finishes
    stop_tactic = StopTactic()
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[id].stop.CopyFrom(stop_tactic)


if __name__ == "__main__":
    # Run the test, -s disables all capturing and -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
