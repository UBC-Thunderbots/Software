import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


def test_two_ai_ball_placement(simulated_test_runner):
    def setup(*args):
        # Initial position is from Blue's perspective
        ball_initial_pos = tbots.Point(2, 2)
        # Final point is going to be from yellow's perspective  (since yellow will be the one placing)
        ball_final_pos = tbots.Point(-3, -2)

        # Setup Bots
        blue_bots = [
            tbots.Point(-2.75, 2.5),
            tbots.Point(-2.75, 1.5),
            tbots.Point(-2.75, 0.5),
            tbots.Point(-2.75, -0.5),
            tbots.Point(-2.75, -1.5),
            tbots.Point(4.6, -3.1),
        ]

        yellow_bots = [
            tbots.Point(1, 0),
            tbots.Point(1, 2.5),
            tbots.Point(1, -2.5),
            tbots.Field.createSSLDivisionBField().enemyGoalCenter(),
            tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
            tbots.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
        ]

        # Game Controller Setup
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )
        # Pass in placement point here - not required for all play tests
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.BALL_PLACEMENT,
            team=Team.YELLOW,
            final_ball_placement_point=ball_final_pos,
        )

        # Force play override here
        blue_play = Play()
        blue_play.name = PlayName.EnemyBallPlacementPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)

        yellow_play = Play()
        yellow_play.name = PlayName.BallPlacementPlay

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

    # TODO- #2783 Validation
    # params just have to be a list of length 1 to ensure the test runs at least once
    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[]],
        test_timeout_s=7.5,
    )


if __name__ == "__main__":
    pytest_main(__file__)
