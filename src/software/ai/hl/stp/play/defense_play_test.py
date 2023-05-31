import sys

import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "yellow_positions",
    [
        [
            tbots.Point(1, 3),
            tbots.Point(1, -0.25),
            tbots.Point(1, -1.25),
            tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
            tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
            tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
        ],
        [
            tbots.Point(-2, 1.05),
            tbots.Point(-3.5, 2),
            tbots.Point(-1.5, 0),
            tbots.Point(-2.3, -1),
            tbots.Point(-3.8, -2),
            tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
        ],
    ],
)
def test_defense_play(simulated_test_runner, yellow_positions):
    def setup(*args):
        # starting point must be Point
        ball_initial_pos = tbots.Point(0.9, 2.85)

        # Setup Bots
        blue_bots = [
            tbots.Point(-4.5, 0),
            tbots.Point(-3, 1.5),
            tbots.Point(-3, 0.5),
            tbots.Point(-3, -0.5),
            tbots.Point(-3, -1.5),
            tbots.Point(-3, -3.0),
        ]

        # Game Controller Setup
        simulated_test_runner.gamecontroller.send_ci_input(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.gamecontroller.send_ci_input(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

        # Force play override here
        blue_play = Play()
        blue_play.name = PlayName.DefensePlay

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            Play, yellow_play
        )

        # Create world state
        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState,
            create_world_state(
                yellow_robot_locations=yellow_positions,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots.Vector(0, 0),
            ),
        )

    # TODO- #2782 Validation
    # params just have to be a list of length 1 to ensure the test runs at least once
    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
