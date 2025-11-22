import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_from_rest import *
from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from software.simulated_tests.or_validation import OrValidation


@pytest.mark.parametrize("is_friendly_test", [True, False])
def test_kickoff_play(simulated_test_runner, is_friendly_test):
    ball_initial_pos = tbots_cpp.Point(0, 0)

    # Setup Bots
    blue_bots = [
        tbots_cpp.Point(-3, 2.5),
        tbots_cpp.Point(-3, 1.5),
        tbots_cpp.Point(-3, 0.5),
        tbots_cpp.Point(-3, -0.5),
        tbots_cpp.Point(-3, -1.5),
        tbots_cpp.Point(-3, -2.5),
    ]

    yellow_bots = [
        tbots_cpp.Point(1, 0),
        tbots_cpp.Point(1, 2.5),
        tbots_cpp.Point(1, -2.5),
        tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXNegYCorner(),
        tbots_cpp.Field.createSSLDivisionBField().enemyDefenseArea().negXPosYCorner(),
    ]

    blue_play = Play()
    yellow_play = Play()

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )

    if is_friendly_test:
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.KICKOFF, team=Team.BLUE
        )
        blue_play.name = PlayName.KickoffFriendlyPlay
        yellow_play.name = PlayName.KickoffEnemyPlay
    else:
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.KICKOFF, team=Team.YELLOW
        )
        blue_play.name = PlayName.KickoffEnemyPlay
        yellow_play.name = PlayName.KickoffFriendlyPlay

    simulated_test_runner.gamecontroller.send_gc_command(
        gc_command=Command.Type.NORMAL_START, team=Team.BLUE
    )

    # Force play override here
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(Play, yellow_play)

    # Create world state
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )

    # Always Validation
    always_validation_sequence_set = [[]]

    ball_moves_at_rest_validation = BallAlwaysMovesFromRest(
        position=tbots_cpp.Point(0, 0), threshold=0.05
    )

    expected_center_circle_or_validation_set = [
        ball_moves_at_rest_validation,
        NumberOfRobotsAlwaysStaysInRegion(
            regions=[tbots_cpp.Field.createSSLDivisionBField().centerCircle()],
            req_robot_cnt=0,
        ),
    ]

    friendly_half = tbots_cpp.Field.createSSLDivisionBField().friendlyHalf()
    friendly_goal = tbots_cpp.Field.createSSLDivisionBField().friendlyGoal()
    center_circle = tbots_cpp.Field.createSSLDivisionBField().centerCircle()

    friendly_regions = [friendly_half, friendly_goal, center_circle]

    if is_friendly_test:
        # this expected_center_circle_or_validation_set version checks
        # that either 0 or 1 robots are in centerCircle OR ball moves from center point
        expected_center_circle_or_validation_set.append(
            NumberOfRobotsAlwaysStaysInRegion(
                regions=[tbots_cpp.Field.createSSLDivisionBField().centerCircle()],
                req_robot_cnt=1,
            )
        )
    else:
        # Checks that 0 robots are in centerCircle OR ball moves from center point
        friendly_regions.remove(center_circle)

    # Checks that there are 6 friendly robots in friendly_regions
    # friendly_regions definition depends on if/else case above
    expected_robot_regions_or_validations_set = [
        ball_moves_at_rest_validation,
        NumberOfRobotsAlwaysStaysInRegion(
            regions=friendly_regions,
            req_robot_cnt=6,
        ),
    ]

    always_validation_sequence_set[0] = [
        OrValidation(expected_center_circle_or_validation_set),
        OrValidation(expected_robot_regions_or_validations_set),
    ]

    eventually_validation_sequence_set = [[]]

    # Eventually Validation
    if is_friendly_test:
        # Checks that ball leaves center point by 0.05 meters within 10 seconds of kickoff
        eventually_validation_sequence_set[0].append(
            BallEventuallyExitsRegion(
                regions=[tbots_cpp.Circle(ball_initial_pos, 0.05)]
            )
        )

    simulated_test_runner.run_test(
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
