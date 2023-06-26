import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.excessive_possession import NeverExcessivelyPossesses
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team


def test_shoot_or_pass_defense_play(simulated_test_runner):
    def setup(*args):
        ball_initial_pos = tbots.Point(0, 0)
        ball_initial_vel = tbots.Vector(0, 0)

        # sets up lines of friendly and enemy robots
        blue_bots = [
            tbots.Point(-2, 2.25),
            tbots.Point(-2, 1.25),
            tbots.Point(-2, 0.25),
            tbots.Point(-2, -0.25),
            tbots.Point(-2, -1.25),
            tbots.Point(-2, -2.25),
        ]

        yellow_bots = [
            tbots.Point(2, 2.25),
            tbots.Point(2, 1.25),
            tbots.Point(2, 0.25),
            tbots.Point(2, -0.25),
            tbots.Point(2, -1.25),
            tbots.Point(2, -2.25),
        ]

        world_state = create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=ball_initial_vel,
        )

        # Game Controller Setup
        simulated_test_runner.gamecontroller.send_ci_input(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.gamecontroller.send_ci_input(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

        blue_play = Play()
        blue_play.name = PlayName.ShootOrPassPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)

        yellow_play = Play()
        yellow_play.name = PlayName.DefensePlay

        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            Play, yellow_play
        )

        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState, world_state
        )

    always_validation_sequence_set = [
        [NeverExcessivelyPossesses(max_possession_time=10)]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_eventually_validation_sequence_set=[[]],
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=60,
    )


if __name__ == "__main__":
    pytest_main(__file__)
