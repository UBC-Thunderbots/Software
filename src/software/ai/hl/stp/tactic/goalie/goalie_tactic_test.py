import sys

import pytest

import software.geom.geometry as geom
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.robots_halt import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.simulated_test_fixture import tactic_runner


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,robot_initial_position",
    [
        # test panic ball very fast in straight line
        (geom.Point(0, 0), geom.Vector(-5, 0), geom.Point(-4, -1)),
        # test panic ball very_fast in diagonal line
        (
            geom.Point(0, 0),
            geom.Vector(-5.5, 0.25),
            geom.Field().friendlyGoalCenter() + geom.Vector(0, -0.5),
        ),
        # test ball very fast misses net
        (geom.Point(0, 0), geom.Vector(-5, 1), geom.Point(-4.5, 0)),
        # test slow ball at sharp angle to friendly goal
        # ball slow inside friendly defense area
        (geom.Point(-4, 0.8), geom.Vector(-0.2, 0), geom.Point(0, 0)),
        # ball slow inside friendly defense area
        (geom.Point(-4, 0.8), geom.Vector(-0.2, 0), geom.Point(0, 2)),
        # ball slow inside friendly defense area
        (geom.Point(-4, 0.8), geom.Vector(-0.2, 0), geom.Point(0, 2)),
        # ball slow inside friendly defense area
        (geom.Point(-4, 0.8), geom.Vector(-0.2, 0), geom.Point(-4, 0)),
        # ball stationary inside friendly defense area
        (geom.Point(-4, 0.0), geom.Vector(0.0, 0), geom.Field().friendlyGoalpostPos()),
        # ball stationary inside no-chip rectangle
        (
            geom.Field().friendlyGoalCenter() + geom.Vector(0.1, 0.1),
            geom.Vector(-0.2, 0),
            geom.Point(-4, -1),
        ),
        # ball fast inside no-chip rectangle but no intersection with goal
        (
            geom.Field().friendlyGoalCenter() + geom.Vector(0.1, 0),
            geom.Vector(0, -0.5),
            geom.Point(-3.5, 1),
        ),
        # ball moving out from inside defense area
        (
            geom.Field().friendlyGoalCenter() + geom.Vector(0.5, 0),
            geom.Vector(0.5, 0),
            geom.Point(-3.5, 0),
        ),
        # ball slow inside no-chip rectangle
        (
            geom.Field().friendlyGoalCenter() + geom.Vector(0.1, 0),
            geom.Vector(0.1, -0.1),
            geom.Point(-3.5, 1),
        ),
        # TODO (#2167): This test fails so disabling for Robocup
        # ball moving into goal from inside defense area
        (
            geom.Field().friendlyGoalCenter() + geom.Vector(0.5, 0),
            geom.Vector(-0.5, 0),
            geom.Point(-3.5, 0),
        ),
        # TODO (#2167): This test fails so disabling for Robocup
        # ball moving up and out of defense area
        (
            geom.Field().friendlyGoalCenter() + geom.Vector(0.3, 0),
            geom.Vector(0, 1),
            geom.Point(-3.5, 0),
        ),
        # TODO (#2167): This test fails so disabling for Robocup
        # ball moving down and out goal from defense area
        (
            geom.Field().friendlyGoalCenter() + geom.Vector(0.3, 0),
            geom.Vector(0, -0.7),
            geom.Point(-3.5, 0),
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
            RobotNeverEntersRegion(regions=[geom.Field().enemyDefenseArea()]),
            BallNeverEntersRegion(regions=[geom.Field().friendlyGoal()]),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            # Goalie should be in the defense area
            RobotsEventuallyHalt(),
            RobotEventuallyEntersRegion(regions=[geom.Field().friendlyDefenseArea()]),
            # BallEventuallyMovesForward(ball_initial_position),
        ]
    ]

    tactic_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-svv"]))
