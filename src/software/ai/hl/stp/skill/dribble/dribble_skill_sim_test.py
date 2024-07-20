import math

import pytest

import software.python_bindings as tbots_cpp
import sys
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.simulated_tests.simulated_test_fixture import *
from proto.message_translation.tbots_protobuf import create_world_state


def test_dribble_skill(simulated_test_runner):

    time.sleep(1)
    # Create world state
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=[],
            blue_robot_locations=[tbots_cpp.Point(-2, 0)],
            ball_location=tbots_cpp.Point(-2, 0),
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    print("making world!")

    time.sleep(1)

    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].dribble_skill.CopyFrom(DribbleSkillTactic(
        dribble_destination=Point(x_meters=-3, y_meters=-2),
        final_orientation=Angle(radians=0),
        excessive_dribbling_allowed=ExcessiveDribblingMode.ALLOWED,
        max_speed_dribble=MaxAllowedSpeedMode.DRIBBLE,
        max_speed_get_possession=MaxAllowedSpeedMode.PHYSICAL_LIMIT
    ))

    simulated_test_runner.set_tactics(params, True)
    print("running test!")
    simulated_test_runner.run_test(
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )
    print("ran test!")


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
