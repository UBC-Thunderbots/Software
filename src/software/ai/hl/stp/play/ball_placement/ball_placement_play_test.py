import pytest
import software.python_bindings as tbots_cpp
from software.py_constants import ENEMY_BALL_PLACEMENT_DISTANCE_METERS

from proto.import_all_protos import *
from proto.ssl_gc_common_pb2 import Team
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.ball_enters_region import (
    BallAlwaysStaysInRegion,
    BallEventuallyEntersRegion,
)
from software.simulated_tests.robot_enters_region import RobotEventuallyExitsRegion
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "ball_start_point, ball_placement_point",
    [
        # test normal ball placement (not edge case)
        (tbots_cpp.Point(2, 2), tbots_cpp.Point(0, 1.5)),
        # test when ball starting point is outside of the goal line
        (tbots_cpp.Point(-4.7, 2.0), tbots_cpp.Point(0, 0.5)),
        # test when ball starting point is outside of the side lines
        (tbots_cpp.Point(-2.0, 3.2), tbots_cpp.Point(0, -0.5)),
        # test when ball starting point is inside of enemy net
        (tbots_cpp.Point(4.0, 0), tbots_cpp.Point(2.0, 2.0)),
        # test when ball starting point is in friendly net
        (tbots_cpp.Point(-4.0, 0), tbots_cpp.Point(-2.0, 2.0)),
    ],
)
def test_two_ai_ball_placement(
    simulated_test_runner, ball_start_point, ball_placement_point
):
    run_ball_placement_scenario(
        simulated_test_runner, ball_start_point, ball_placement_point
    )


@pytest.mark.parametrize(
    "ball_start_point, ball_placement_point",
    [
        # TODO: Remove these ball placement tests until #3561 is resolved
        # # 2023 RoboCup ball placement scenarios
        # # Scenario 1
        # (tbots_cpp.Point(-0.2, -2.8), tbots_cpp.Point(-0.2, 2.8)),
        # # Scenario 2
        # (tbots_cpp.Point(-3.5, -2.25), tbots_cpp.Point(0, 0)),
        # # Scenario 3
        # (tbots_cpp.Point(-1.5, -2.25), tbots_cpp.Point(-0.2, -2.8)),
        # # Scenario 4
        # (tbots_cpp.Point(-4.4, -2.9), tbots_cpp.Point(-0.2, 2.8)),
        # # Scenario 5
        # (tbots_cpp.Point(-0.5, -0), tbots_cpp.Point(-4.3, 2.8)),
        # # Scenario 6
        # (tbots_cpp.Point(-1, -3.15), tbots_cpp.Point(-3.5, -2.8)),
        # # Scenario 7
        # (tbots_cpp.Point(-1, 3.15), tbots_cpp.Point(-3.5, 2.8)),
        # # Scenario 8
        # (tbots_cpp.Point(-4.45, -0.1), tbots_cpp.Point(-0.5, 2.8)),
    ],
)
def test_robocup_technical_challenge_placement(
    simulated_test_runner, ball_start_point, ball_placement_point
):
    run_ball_placement_scenario(
        simulated_test_runner, ball_start_point, ball_placement_point, blue_only=True
    )


def ball_placement_play_setup(
    ball_start_point, ball_placement_point, simulated_test_runner, blue_only
):
    """Set up ball placement test by initializing bot positions, ball placement targets, and test settings

    :param ball_start_point: Initial point of the ball
    :param ball_placement_point: Target point of the ball
    :param simulated_test_runner: Simulated test runner
    :param blue_only: If True, only the blue team is active; the yellow team is ignored.
    """
    # Setup blue robots
    blue_bots = [
        tbots_cpp.Point(-2.75, 1.5),
        tbots_cpp.Point(-0.0, 0.0),
        tbots_cpp.Point(-2.75, -0.5),
        tbots_cpp.Field.createSSLDivisionBField().friendlyGoalCenter(),
        tbots_cpp.Field.createSSLDivisionBField()
        .friendlyDefenseArea()
        .negXNegYCorner(),
        tbots_cpp.Field.createSSLDivisionBField()
        .friendlyDefenseArea()
        .negXPosYCorner(),
    ]

    yellow_bots = []

    # Optionally skip yellow robots entirely
    if not blue_only:
        yellow_bots = [
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(1, 2.5),
            tbots_cpp.Point(1, -2.5),
            tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
            tbots_cpp.Field.createSSLDivisionBField()
            .enemyDefenseArea()
            .negXNegYCorner(),
            tbots_cpp.Field.createSSLDivisionBField()
            .enemyDefenseArea()
            .negXPosYCorner(),
        ]

    # Game Controller Setup
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    # Pass in placement point here - not required for all play tests
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.BALL_PLACEMENT,
        team=Team.BLUE,
        final_ball_placement_point=ball_placement_point,
    )

    # Force play override here
    blue_play = Play()
    blue_play.name = PlayName.BallPlacementPlay

    yellow_play = Play()
    yellow_play.name = PlayName.HaltPlay

    simulated_test_runner.set_play(blue_play, is_friendly=True)
    if not blue_only:
        simulated_test_runner.set_play(yellow_play, is_friendly=False)

    # Create world state
    simulated_test_runner.set_world_state(
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_start_point,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
    )


def run_ball_placement_scenario(
    simulated_test_runner, ball_start_point, ball_placement_point, blue_only=False
):
    """Runs a ball placement test scenario with the specified parameters.

    :param simulated_test_runner: The test runner used to simulate robot and ball behavior.
    :param ball_start_point: The initial position of the ball.
    :param ball_placement_point: The target position where the ball should be placed.
    :param blue_only: If True, only the blue team is active; the yellow team is ignored.
    """
    # Placement Eventually Validation
    placement_eventually_validation_sequence_set = [
        [
            # Ball should arrive within 0.15m of placement point
            BallEventuallyEntersRegion(
                regions=[tbots_cpp.Circle(ball_placement_point, 0.15)]
            ),
        ]
    ]

    # Drop Ball Always Validation
    drop_ball_always_validation_sequence_set = [
        [
            BallAlwaysStaysInRegion(
                regions=[tbots_cpp.Circle(ball_placement_point, 0.15)]
            ),
        ]
    ]

    # Drop Ball Eventually Validation
    # Non free kick after ball placement, the robot must be 0.5 away from the ball after the placement
    # See detailed rules here: https://robocup-ssl.github.io/ssl-rules/sslrules.html#_ball_placement
    drop_ball_eventually_validation_sequence_set = [
        [
            RobotEventuallyExitsRegion(
                regions=[
                    tbots_cpp.Circle(
                        ball_placement_point, ENEMY_BALL_PLACEMENT_DISTANCE_METERS
                    )
                ]
            ),
        ]
    ]

    simulated_test_runner.run_test(
        setup=lambda test_setup_arg: ball_placement_play_setup(
            test_setup_arg["ball_start_point"],
            test_setup_arg["ball_placement_point"],
            simulated_test_runner,
            blue_only,
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
        test_timeout_s=[15],
    )

    simulated_test_runner.run_test(
        # setup argument isn't passed to preserve world state from previous test run
        inv_always_validation_sequence_set=drop_ball_always_validation_sequence_set,
        inv_eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
        ag_always_validation_sequence_set=drop_ball_always_validation_sequence_set,
        ag_eventually_validation_sequence_set=drop_ball_eventually_validation_sequence_set,
        test_timeout_s=[5],
    )


if __name__ == "__main__":
    pytest_main(__file__)
