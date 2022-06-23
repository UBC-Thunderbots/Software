import pytest

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_speed_threshold import *
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.pytest_main import pytest_main
from proto.message_translation.tbots_protobuf import create_world_state, parse_world_state
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.simulated_tests.field_tests.field_test_fixture import *

ball_initial_position = tbots.Point(-2.5, 0)
kick_velocity = tbots.Vector(6, 0)

rob_pos = ball_initial_position - (kick_velocity.normalize() * 0.5)


def test_kick(
    field_test_runner,
):

    print("STARTED RUNNING TEST")

    ball_initial_position = tbots.Point(-2.5, 0)
    kick_velocity = tbots.Vector(6, 0)

    rob_pos = tbots.Point(0.5, 1)

    # ybots, bbots, ball_initial_position, kick_velocity = parse_world_state(field_test_runner.initial_worldstate)
    # rob_pos = bbots[0]
    field_test_runner.set_worldState(
        create_world_state(
            [],
            blue_robot_locations=[rob_pos],
            ball_location=ball_initial_position,
            ball_velocity=tbots.Vector(0, 0),
        ),
    )

    exit(0)

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    kick_origin = Point(
        x_meters=ball_initial_position.x(), y_meters=ball_initial_position.y()
    )

    params.assigned_tactics[0].kick.CopyFrom(
        KickTactic(
            kick_origin=kick_origin,
            kick_direction=Angle(radians=0.0),
            kick_speed_meters_per_second=kick_velocity.length(),
        )
    )

    # field_test_runner.blue_full_system_proto_unix_io.send_proto(
    #     AssignedTacticPlayControlParams, params
    # )
    #
    # # Setup no tactics on the enemy side
    # params = AssignedTacticPlayControlParams()
    # field_test_runner.yellow_full_system_proto_unix_io.send_proto(
    #     AssignedTacticPlayControlParams, params
    # )

    # Always Validation
    always_validation_sequence_set = []

    # Eventually Validation
    eventually_validation_sequence_set = [
        [BallEntersRegion([tbots.Field.createSSLDivisionBField().centerCircle()])]
    ]

    field_test_runner.run_test(
        test_timeout_s=5,
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    print("ENTERED")
    pytest_main(__file__)
