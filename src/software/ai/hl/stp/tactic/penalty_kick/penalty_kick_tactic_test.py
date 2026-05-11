import pytest

import software.python_bindings as tbots_cpp
from proto.import_all_protos import PenaltyKickTactic

from software.simulated_tests.validation.friendly_team_scored import (
    FriendlyTeamEventuallyScored,
)
from software.simulated_tests.validation.excessive_dribbling import (
    NeverExcessivelyDribbles,
)
from software.simulated_tests.validation.ball_moves_in_direction import (
    BallAlwaysMovesForward,
)
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "enemy_robot_location, enemy_robot_velocity",
    [
        # enemy robot stationary at centre of goal
        (
            tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
            tbots_cpp.Vector(0, 0),
        ),
        # enemy robot stationary left of net
        (
            tbots_cpp.Field.createSSLDivisionBField().enemyGoalpostNeg(),
            tbots_cpp.Vector(0, 0),
        ),
        # enemy robot stationary right of net
        (
            tbots_cpp.Field.createSSLDivisionBField().enemyGoalpostPos(),
            tbots_cpp.Vector(0, 0),
        ),
        # enemy robot left of net but moving right
        (
            tbots_cpp.Field.createSSLDivisionBField().enemyGoalpostNeg(),
            tbots_cpp.Vector(0, 1.2),
        ),
        # enemy robot right of net but moving left
        (
            tbots_cpp.Field.createSSLDivisionBField().enemyGoalpostPos(),
            tbots_cpp.Vector(0, -1.2),
        ),
    ],
)
@pytest.mark.skip(
    "Disabling this test because of poor dribbling controls, does not consistently score goal. TODO (#2232)"
)
def test_penalty_kick(
    enemy_robot_location, enemy_robot_velocity, simulated_test_runner
):
    field = tbots_cpp.Field.createSSLDivisionBField()
    ball_initial_pos = field.friendlyPenaltyMark()

    def setup(*args):
        shooter_position = ball_initial_pos - tbots_cpp.Vector(0.1, 0)

        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[shooter_position],
                yellow_robot_locations=[enemy_robot_location],
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.set_tactics(blue_tactics={0: PenaltyKickTactic()})

    eventually_validation_sequence_set = [
        [
            FriendlyTeamEventuallyScored(),
        ]
    ]

    # For RoboCup SSL rules: ball never moves backward
    always_validation_sequence_set = [
        [BallAlwaysMovesForward(ball_initial_pos), NeverExcessivelyDribbles()]
    ]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
