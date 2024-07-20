import math

import pytest

import software.python_bindings as tbots_cpp
import sys
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *


def test_dribble_skill(field_test_runner):
    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    if len(world.friendly_team.team_robots) == 0:
        raise Exception("The first world received had no robots in it!")

    print("Here are the robots:")
    print(
        [
            robot.current_state.global_position
            for robot in world.friendly_team.team_robots
        ]
    )

    id = world.friendly_team.team_robots[0].id
    print(f"Running test on robot {id}")

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[id].dribble_skill.CopyFrom(DribbleSkillTactic(
        dribble_destination=Point(x_meters=-3, y_meters=0),
        final_orientation=Angle(radians=0),
        excessive_dribbling_allowed=ExcessiveDribblingMode.ALLOWED,
        max_speed_dribble=MaxAllowedSpeedMode.DRIBBLE,
        max_speed_get_possession=MaxAllowedSpeedMode.PHYSICAL_LIMIT
    ))

    field_test_runner.set_tactics(params, True)
    field_test_runner.run_test(
        always_validation_sequence_set=[[]],
        eventually_validation_sequence_set=[[]],
        test_timeout_s=99999,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
