import software.python_bindings as tbots_cpp
import sys
from proto.ssl_gc_state_pb2 import Command
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *

from software.simulated_tests.simulated_test_fixture import *
from software.logger.logger import createLogger

logger = createLogger(__name__)


# this field test will start a ball placement referee command to the placement_point.
def test_ball_placement(field_test_runner):
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


_
if __name__ == "__main__":
    # Run the test, -s disables all capturing and -vv increases verbosity
    sys.exit(pytest_main(__file__))
