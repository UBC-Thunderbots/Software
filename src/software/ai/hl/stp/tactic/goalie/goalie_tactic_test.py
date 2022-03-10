import logging
import sys

import pytest

import software.geom.geometry as tbots_geom
from proto.primitive_pb2 import MaxAllowedSpeedMode
from proto.tactic_pb2 import AssignedTacticPlayControlParams, GoalieTactic, Tactic
from software.simulated_tests.eventually_validation.robot_enters_region import (
    RobotEntersRegion,
)
from software.simulated_tests.simulated_test_fixture import tactic_runner


@pytest.mark.parametrize(
    "goalie_starting_position,ball_starting_position",
    [
        ((4.2, 0), (-2, 1)),
        ((4.2, 0.4), (-2, -1)),
        ((4.2, -0.4), (-2, 0.1)),
        ((-4.2, 0.4), (-2, -1)),
        ((-4.2, -0.4), (-2, 0.1)),
        ((-4.2, 0.2), (-2, -0.1)),
        ((-4.2, -0.2), (-2, 1)),
        ((-4.2, 0.2), (-2, -0.1)),
        ((-4.2, -0.5), (-2, 2)),
        ((-4.2, -0.5), (-2, -2)),
    ],
)
def test_goalie_blocks_shot(
    goalie_starting_position, ball_starting_position, tactic_runner
):
    # Setup Robot
    tactic_runner.simulator.setup_yellow_robots([goalie_starting_position])

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].goalie.CopyFrom(
        GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )
    tactic_runner.yellow_full_system.send_tactic_override(params)

    # Setup ball with initial velocity using our software/geom
    tactic_runner.simulator.setup_ball(
        ball_position=tbots_geom.Point(*ball_starting_position),
        ball_velocity=(
            tbots_geom.Field().friendlyGoalCenter()
            - tbots_geom.Vector(ball_starting_position[0], ball_starting_position[1])
        )
        .toVector()
        .normalize(5),  # TODO we should try a range of speeds
    )

    # Validation
    eventually_sequences = [
        [
            # Goalie should be in the defense area
            RobotEntersRegion(regions=[tbots_geom.Field().friendlyDefenseArea()])
        ]
    ]

    tactic_runner.run_test(eventually_validation_sequence_set=eventually_sequences)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-svv"]))
