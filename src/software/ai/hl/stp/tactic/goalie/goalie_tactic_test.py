import sys

import pytest

import software.python_bindings as tbots
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.friendly_has_ball_possession import *
from software.simulated_tests.ball_speed_threshold import *
from software.simulated_tests.robot_speed_threshold import *
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import tactic_runner


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,robot_initial_position",
    [
        # test panic ball very fast in straight line
        (tbots.Point(0, 0), tbots.Vector(-5, 0), tbots.Point(-4, -1)),
        # test panic ball very_fast in diagonal line
        (
            tbots.Point(0, 0),
            tbots.Vector(-5.5, 0.25),
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0, -0.5),
        ),
        # test ball very fast misses net
        (tbots.Point(0, 0), tbots.Vector(-5, 1), tbots.Point(-4.5, 0)),
        # test slow ball at sharp angle to friendly goal
        # ball slow inside friendly defense area
        (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 0)),
        # ball slow inside friendly defense area
        (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 2)),
        # ball slow inside friendly defense area
        (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 2)),
        # ball slow inside friendly defense area
        (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(-4, 0),),
        # ball stationary inside friendly defense area
        (
            tbots.Point(-4, 0.0),
            tbots.Vector(0.0, 0),
            tbots.Field.createSSLDivisionBField().friendlyGoalpostPos(),
        ),
        # ball stationary inside no-chip rectangle
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.1, 0.1),
            tbots.Vector(-0.2, 0),
            tbots.Point(-4, -1),
        ),
        # ball fast inside no-chip rectangle but no intersection with goal
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.1, 0),
            tbots.Vector(0, -0.5),
            tbots.Point(-3.5, 1),
        ),
        # ball moving out from inside defense area
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.5, 0),
            tbots.Vector(0.5, 0),
            tbots.Point(-3.5, 0),
        ),
        # ball slow inside no-chip rectangle
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.1, 0),
            tbots.Vector(0.1, -0.1),
            tbots.Point(-3.5, 1),
        ),
        # TODO (#2167): This test fails so disabling for Robocup
        # ball moving into goal from inside defense area
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.5, 0),
            tbots.Vector(-0.5, 0),
            tbots.Point(-3.5, 0),
        ),
        # TODO (#2167): This test fails so disabling for Robocup
        # ball moving up and out of defense area
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.3, 0),
            tbots.Vector(0, 1),
            tbots.Point(-3.5, 0),
        ),
        # TODO (#2167): This test fails so disabling for Robocup
        # ball moving down and out goal from defense area
        (
            tbots.Field.createSSLDivisionBField().friendlyGoalCenter()
            + tbots.Vector(0.3, 0),
            tbots.Vector(0, -0.7),
            tbots.Point(-3.5, 0),
        ),
    ],
)
def test_goalie_blocks_shot(
    ball_initial_position, ball_initial_velocity, robot_initial_position, tactic_runner
):
    # Setup Robot
    tactic_runner.simulator.setup_yellow_robots([robot_initial_position])

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].goalie.CopyFrom(
        GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )
    tactic_runner.yellow_full_system.send_tactic_override(params)

    # Setup ball with initial velocity using our software/geom
    tactic_runner.simulator.setup_ball(
        ball_position=ball_initial_position, ball_velocity=ball_initial_velocity
    )

    # Always Validation
    always_validation_sequence_set = [
        [
            RobotNeverEntersRegion(
                regions=[tbots.Field.createSSLDivisionBField().enemyDefenseArea()]
            ),
            BallNeverEntersRegion(
                regions=[tbots.Field.createSSLDivisionBField().friendlyGoal()]
            ),
            NeverExcessivelyDribbling(),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            # Goalie should be in the defense area
            RobotEventuallyEntersRegion(
                regions=[tbots.Field.createSSLDivisionBField().friendlyDefenseArea()]
            ),
            # FriendlyEventuallyHasBallPossession(),
            # BallSpeedEventuallyAtOrAboveThreshold(1),
            # RobotSpeedEventuallyAtOrAboveThreshold(1),
            # BallEventuallyMovesForward(ball_initial_position),
        ]
    ]

    tactic_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-svv"]))
