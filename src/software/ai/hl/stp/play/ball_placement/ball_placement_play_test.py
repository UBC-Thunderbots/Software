import pytest

import software.python_bindings as tbots
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_stops_in_region import *
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


def ball_placement_play_setup(
    ball_start_point, ball_placement_point, simulated_test_runner
):
    # Setup Bots
    blue_bots = [
        tbots.Point(-2.75, 1.5),
        tbots.Point(-0.0, 0.0),
        tbots.Point(-2.75, -0.5),
        tbots.Field.createSSLDivisionBField().friendlyGoalCenter(),
        tbots.Field.createSSLDivisionBField().friendlyDefenseArea().negXNegYCorner(),
        tbots.Field.createSSLDivisionBField().friendlyDefenseArea().negXPosYCorner(),
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
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    # Pass in placement point here - not required for all play tests
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.BALL_PLACEMENT,
        team=Team.BLUE,
        final_ball_placement_point=ball_placement_point,
    )

    # Force play override here
    blue_play = Play()
    blue_play.name = PlayName.BallPlacementPlay

    # TODO (#3019): Reenable enemy ai after enemy ball placement is fixed
    yellow_play = Play()
    yellow_play.name = PlayName.HaltPlay

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(Play, yellow_play)

    # Create world state
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_start_point,
            ball_velocity=tbots.Vector(0, 0),
        ),
    )


# TODO (#2599): Remove Duration parameter from test
# TODO (#2690): Robot gets stuck in corner of defense area
@pytest.mark.parametrize(
    "ball_start_point, ball_placement_point",
    [
        # test normal ball placement (not edge case)
        (tbots.Point(2, 2), tbots.Point(0, 1.5)),
        # test when ball starting point is outside of the goal line
        (tbots.Point(-4.7, 1.6), tbots.Point(0, 0.5)),
        # test when ball starting point is outside of the side lines
        (tbots.Point(-2.0, 3.2), tbots.Point(0, -0.5)),
        # test when ball placement point is inside of the friendly defense area
        (tbots.Point(-3.6, 0.0), tbots.Point(0, -1.5)),
    ],
)
def test_two_ai_ball_placement(
    simulated_test_runner, ball_start_point, ball_placement_point
):
    # Placement Eventually Validation
    placement_eventually_validation_sequence_set = [
        [
            # Ball should arrive within 0.15m of placement point
            BallEventuallyEntersRegion(
                regions=[tbots.Circle(ball_placement_point, 0.15)]
            ),
            RobotEventuallyEntersRegion(
                regions=[tbots.Circle(ball_placement_point, 0.15)]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=lambda test_setup_arg: ball_placement_play_setup(
            test_setup_arg["ball_start_point"],
            test_setup_arg["ball_placement_point"],
            simulated_test_runner,
        ),
        params=[
            {
                "ball_start_point": ball_start_point,
                "ball_placement_point": ball_placement_point,
            }
        ],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=placement_eventually_validation_sequence_set,
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=placement_eventually_validation_sequence_set,
        test_timeout_s=[20],
    )

    # Drop Ball Always Validation
    drop_ball_always_validation_sequence_set = [
        [BallAlwaysStaysInRegion(regions=[tbots.Circle(ball_placement_point, 0.1)]),]
    ]

    # Drop Ball Eventually Validation
    # Non free kick after ball placement, the robot must be 0.5 away from the ball after the placement
    # See detailed rules here: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement
    drop_ball_eventually_validation_sequence_set = [
        [
            # Ball should arrive within 5cm of placement point
            BallEventuallyStopsInRegion(
                regions=[tbots.Circle(ball_placement_point, 0.05)]
            ),
            RobotEventuallyExitsRegion(
                regions=[tbots.Circle(ball_placement_point, 0.5)]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        # setup argument isn't passed to preserve world state from previous test run
        inv_always_validation_sequence_set=drop_ball_always_validation_sequence_set,
        inv_eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
        ag_always_validation_sequence_set=drop_ball_always_validation_sequence_set,
        ag_eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
        test_timeout_s=[10],
    )


if __name__ == "__main__":
    pytest_main(__file__)
