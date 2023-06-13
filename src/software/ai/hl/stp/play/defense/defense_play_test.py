import sys

import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import simulated_test_runner
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "blue_bots,yellow_bots",
    [
        (
            [
                tbots.Point(-3, 1.5),
                tbots.Point(-3, 0.5),
                tbots.Point(-3, -0.5),
                tbots.Point(-3, -1.5),
                tbots.Point(-3, 1),
                tbots.Point(-3, 0.75),
            ],
            [
                tbots.Point(1, -0.25),
                tbots.Point(1, -1.25),
                tbots.Point(2, -0.25),
                tbots.Point(2, -1.25),
                tbots.Point(2, -1),
            ],
        )
    ],
)
def test_defense_play(simulated_test_runner, blue_bots, yellow_bots):
    def setup(*args):
        # Starting point must be Point
        ball_initial_pos = tbots.Point(0.9, 2.85)
        # Placement point must be Vector2 to work with game controller
        tbots.Point(-3, -2)

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
        yellow_play.name = PlayName.ShootOrPassPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            Play, yellow_play
        )

        # Create world state
        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState,
            create_world_state(
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots.Vector(0, 0),
            ),
        )

    simulated_test_runner.run_test(
        setup=setup,
        params=[0, 1, 2, 3, 4],  # The aggregate test runs 5 times
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[
            [
                BallNeverEntersRegion(
                    regions=[tbots.Field.createSSLDivisionBField().friendlyGoal()]
                )
            ]
        ],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=60,
    )


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
