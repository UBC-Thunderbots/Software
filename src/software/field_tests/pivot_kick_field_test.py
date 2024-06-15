import pytest

import software.python_bindings as tbots_cpp
import sys
import math
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *

from software.simulated_tests.simulated_test_fixture import *
from software.logger.logger import create_logger
from software.simulated_tests.robot_enters_region import RobotEventuallyEntersRegion
from proto.message_translation.tbots_protobuf import create_world_state

logger = create_logger(__name__)


def test_pivot_kick(field_test_runner):

    id = 5

    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    print("Here are the robots:")
    print(
        [
            robot.current_state.global_position
            for robot in world.friendly_team.team_robots
        ]
    )

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[id].pivot_kick.CopyFrom(
        PivotKickTactic(
            kick_origin=Point(x_meters=-1.13, y_meters=0.75),
            kick_direction=Angle(radians=-math.pi / 2),
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=5.0),
        )
    )

    field_test_runner.set_tactics(params, True)
    field_test_runner.run_test(
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=15,
    )
    # Send a stop tactic after the test finishes
    stop_tactic = StopTactic()
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[id].stop.CopyFrom(stop_tactic)


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
