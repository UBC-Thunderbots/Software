import pytest

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.friendly_team_scored import *
from software.simulated_tests.ball_moves_from_rest import *
from software.simulated_tests.friendly_has_ball_possession import (
    FriendlyEventuallyHasBallPossession,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def setup_free_kick_play(
    simulated_test_runner, blue_bots, yellow_bots, ball_initial_pos
):
    """Set world state and issue a blue-team direct free kick."""
    simulated_test_runner.set_world_state(
        create_world_state(
            yellow_robot_locations=yellow_bots,
            blue_robot_locations=blue_bots,
            ball_location=ball_initial_pos,
            ball_velocity=tbots_cpp.Vector(0, 0),
        )
    )
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.NORMAL_START, team=Team.BLUE
    )
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.DIRECT, team=Team.BLUE
    )

    blue_play = Play()
    blue_play.name = PlayName.FreeKickPlay

    yellow_play = Play()
    yellow_play.name = PlayName.HaltPlay

    simulated_test_runner.set_play(blue_play, is_friendly=True)
    simulated_test_runner.set_play(yellow_play, is_friendly=False)


# Test 1: Direct Shot
# FSM path: SetupPositionState -> ShootState -> X
# Triggers: setupDone=True, shotFound=True, shotDone=True
def test_free_kick_play_direct_shot(simulated_test_runner):
    """Ball in enemy half with no enemies blocking the goal so kicker shoots directly.
    Validates FriendlyTeamEventuallyScored.
    """
    field = tbots_cpp.Field.createSSLDivisionBField()
    ball_initial_pos = tbots_cpp.Point(1.5, 0.0)

    blue_bots = [
        tbots_cpp.Point(-4.5, 0),
        tbots_cpp.Point(1.3, 0.3),
        tbots_cpp.Point(-1.0, 1.5),
        tbots_cpp.Point(-1.0, -1.5),
        tbots_cpp.Point(0.5, 2.0),
        tbots_cpp.Point(0.5, -2.0),
    ]

    yellow_bots = [
        tbots_cpp.Point(0.0, 3.0),
        tbots_cpp.Point(0.0, -3.0),
        tbots_cpp.Point(-2.0, 2.0),
        tbots_cpp.Point(-2.0, -2.0),
        tbots_cpp.Point(-3.0, 1.0),
        tbots_cpp.Point(-3.0, -1.0),
    ]

    def setup(*args):
        setup_free_kick_play(
            simulated_test_runner, blue_bots, yellow_bots, ball_initial_pos
        )

    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[
            [BallAlwaysStaysInRegion(regions=[field.fieldBoundary()])]
        ],
        inv_eventually_validation_sequence_set=[[FriendlyTeamEventuallyScored()]],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[[FriendlyTeamEventuallyScored()]],
        test_timeout_s=15,
    )


# Test 2: Pass Completes
# FSM path: SetupPositionState -> AttemptPassState -> PassState -> X
# Triggers: setupDone=True, shotFound=False, passFound=True, passDone=True
def test_free_kick_play_pass_completes(simulated_test_runner):
    """Goal is blocked by enemy robots and blue receivers are open in the enemy half. Pass will be made.
    Validates that a receiver eventually gets possession.
    """
    field = tbots_cpp.Field.createSSLDivisionBField()
    ball_initial_pos = tbots_cpp.Point(-0.5, 0.0)

    blue_bots = [
        tbots_cpp.Point(-4.5, 0),
        tbots_cpp.Point(-0.7, 0.2),  # kicker near ball
        tbots_cpp.Point(2.5, 1.5),
        tbots_cpp.Point(2.5, -1.5),
        tbots_cpp.Point(-2.0, 1.0),
        tbots_cpp.Point(-2.0, -1.0),
    ]

    yellow_bots = [
        field.enemyGoalCenter(),
        field.enemyDefenseArea().negXNegYCorner(),
        field.enemyDefenseArea().negXPosYCorner(),
        tbots_cpp.Point(3.5, 0.5),
        tbots_cpp.Point(3.5, -0.5),
        tbots_cpp.Point(3.5, 0.0),
    ]

    def setup(*args):
        setup_free_kick_play(
            simulated_test_runner, blue_bots, yellow_bots, ball_initial_pos
        )

    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[
            [BallAlwaysStaysInRegion(regions=[field.fieldBoundary()])]
        ],
        inv_eventually_validation_sequence_set=[
            [FriendlyEventuallyHasBallPossession(tolerance=0.15)]
        ],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [FriendlyEventuallyHasBallPossession(tolerance=0.15)]
        ],
        test_timeout_s=20,
    )


# Test 3: Abort Pass and Retry
# FSM path: SetupPositionState -> AttemptPassState -> PassState
#           -> AttemptPassState -> ... -> X (chip or second pass)
# Triggers: passFound=True, shouldAbortPass=True
# shouldAbortPass fires when ratePass() drops below abs_min_pass_score (0.05)


def test_free_kick_play_abort_pass_and_retry(simulated_test_runner):
    """Ball in friendly half so shotFound is reliably False.
    A yellow robot sits directly on the pass lane from the ball to receiver B,
    so ratePass() scores the found pass below abs_min_pass_score (0.05).
    Validates ball eventually moves via chip or a cleared second-attempt pass.
    """
    field = tbots_cpp.Field.createSSLDivisionBField()
    # (-0.5, 0) shot angle to enemy goal about 4 < min_open_angle_for_shot_deg (6)
    ball_initial_pos = tbots_cpp.Point(-0.5, 0.0)

    blue_bots = [
        tbots_cpp.Point(-4.5, 0),
        tbots_cpp.Point(-0.7, 0.2),  # kicker near ball
        tbots_cpp.Point(2.0, 0.5),  # receiver A in enemy half
        tbots_cpp.Point(0.5, 2.0),  # receiver B near midfield
        tbots_cpp.Point(-2.0, 1.5),
        tbots_cpp.Point(-2.0, -1.5),
    ]

    yellow_bots = [
        field.enemyGoalCenter(),
        field.enemyDefenseArea().negXNegYCorner(),
        field.enemyDefenseArea().negXPosYCorner(),
        tbots_cpp.Point(0.0, 1.0),  # directly on pass lane to receiver B
        tbots_cpp.Point(3.5, 0.5),
        tbots_cpp.Point(3.5, -0.5),
    ]

    def setup(*args):
        setup_free_kick_play(
            simulated_test_runner, blue_bots, yellow_bots, ball_initial_pos
        )

    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[
            [BallAlwaysStaysInRegion(regions=[field.fieldBoundary()])]
        ],
        inv_eventually_validation_sequence_set=[
            [BallEventuallyMovesFromRest(position=ball_initial_pos)]
        ],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [BallEventuallyMovesFromRest(position=ball_initial_pos)]
        ],
        test_timeout_s=25,
    )


# Test 4: Timeout -> Chip
# FSM path: SetupPositionState -> AttemptPassState -> ChipState -> X
# Triggers: setupDone=True, shotFound=False, passFound=False (timeout), chipDone=True
def test_free_kick_play_chip_on_timeout(simulated_test_runner):
    """Goal is blocked and all receiver zones are covered by enemies. passFound never becomes True,
    so timeExpired fires and the FSM chips.
    Validates that the ball eventually moves from rest via the chip.
    """
    field = tbots_cpp.Field.createSSLDivisionBField()
    ball_initial_pos = tbots_cpp.Point(-1.0, 0.0)

    blue_bots = [
        tbots_cpp.Point(-4.5, 0),
        tbots_cpp.Point(-1.2, 0.2),
        tbots_cpp.Point(-3.0, 1.5),
        tbots_cpp.Point(-3.0, -1.5),
        tbots_cpp.Point(-3.5, 0.5),
        tbots_cpp.Point(-3.5, -0.5),
    ]

    yellow_bots = [
        field.enemyGoalCenter(),
        field.enemyDefenseArea().negXNegYCorner(),
        field.enemyDefenseArea().negXPosYCorner(),
        tbots_cpp.Point(1.0, 0.0),
        tbots_cpp.Point(1.0, 1.5),
        tbots_cpp.Point(1.0, -1.5),
    ]

    def setup(*args):
        setup_free_kick_play(
            simulated_test_runner, blue_bots, yellow_bots, ball_initial_pos
        )

    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[[]],
        inv_eventually_validation_sequence_set=[
            [BallEventuallyMovesFromRest(position=ball_initial_pos)]
        ],
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=[
            [BallEventuallyMovesFromRest(position=ball_initial_pos)]
        ],
        test_timeout_s=20,
    )


# Test 5: Near-Sideline
# Exercises all FSM paths with ball at field edges
@pytest.mark.parametrize(
    "ball_initial_pos,must_score",
    [
        # Enemy half, near sideline for direct shot
        (tbots_cpp.Point(1.5, 0.0), True),
        # Friendly half, near sideline for pass or chip
        (tbots_cpp.Point(-1.5, -2.75), False),
        # Near corner for pass or chip
        (tbots_cpp.Point(1.5, -3.0), False),
        # Enemy half, center for direct shot likely
        (tbots_cpp.Point(1.5, -0.5), True),
    ],
)
def test_free_kick_play_near_sideline(
    simulated_test_runner, ball_initial_pos, must_score
):
    """Parametrized test covering ball positions at and near field edges. Requires score if must_score is True, otherwise
    moving the ball will pass the test.
    """
    field = tbots_cpp.Field.createSSLDivisionBField()

    blue_bots = [
        tbots_cpp.Point(-4.5, 0),
        tbots_cpp.Point(-3.0, 1.5),
        tbots_cpp.Point(-3.0, 0.5),
        tbots_cpp.Point(-3.0, -0.5),
        tbots_cpp.Point(-3.0, -1.5),
        tbots_cpp.Point(4.0, -2.5),
    ]
    yellow_bots = [
        tbots_cpp.Point(1.0, 0),
        tbots_cpp.Point(1.0, 2.5),
        tbots_cpp.Point(1.0, -2.5),
        field.enemyGoalCenter(),
        field.enemyDefenseArea().negXNegYCorner(),
        field.enemyDefenseArea().negXPosYCorner(),
    ]

    def setup(*args):
        setup_free_kick_play(
            simulated_test_runner, blue_bots, yellow_bots, ball_initial_pos
        )

    if must_score:
        inv_eventually = [[FriendlyTeamEventuallyScored()]]
        ag_eventually = [[FriendlyTeamEventuallyScored()]]
    else:
        inv_eventually = [[BallEventuallyMovesFromRest(position=ball_initial_pos)]]
        ag_eventually = [[BallEventuallyMovesFromRest(position=ball_initial_pos)]]

    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[
            [BallAlwaysStaysInRegion(regions=[field.fieldBoundary()])]
        ],
        inv_eventually_validation_sequence_set=inv_eventually,
        ag_always_validation_sequence_set=[[]],
        ag_eventually_validation_sequence_set=ag_eventually,
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
