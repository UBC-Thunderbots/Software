import pytest
import software.python_bindings as tbots_cpp

from proto.import_all_protos import *

from proto.play_pb2 import Play, PlayName
from proto.ssl_gc_common_pb2 import Team

from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.pytest_validations.friendly_team_scored import (
    FriendlyTeamEventuallyScored,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


def free_kick_play_setup(
    blue_bots, yellow_bots, ball_initial_pos, play_name, simulated_test_runner
):
    simulated_test_runner.set_world_state(
        create_world_state(
            blue_robot_locations=blue_bots,
            yellow_robot_locations=yellow_bots,
            ball_location=ball_initial_pos,
            ball_velocity=tbots_cpp.Vector(0, 0),
        ),
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
    blue_play.name = play_name

    yellow_play = Play()
    yellow_play.name = PlayName.HaltPlay

    simulated_test_runner.set_play(blue_play, is_friendly=True)
    simulated_test_runner.set_play(yellow_play, is_friendly=False)


# We want to test friendly half, enemy half, and at the border of the field
@pytest.mark.parametrize(
    "ball_initial_pos,must_score",
    [
        (tbots_cpp.Point(1.5, -2.75), False),
        (tbots_cpp.Point(-1.5, -2.75), False),
        (tbots_cpp.Point(1.5, -3), False),
        (tbots_cpp.Point(1.5, -0.5), True),
    ],
)
def test_free_kick_play_friendly(ball_initial_pos, must_score, simulated_test_runner):
    def setup(*args):
        free_kick_play_setup(
            blue_bots=[
                tbots_cpp.Point(-4.5, 0),
                tbots_cpp.Point(-3, 1.5),
                tbots_cpp.Point(-3, 0.5),
                tbots_cpp.Point(-3, -0.5),
                tbots_cpp.Point(-3, -1.5),
                tbots_cpp.Point(4, -2.5),
            ],
            yellow_bots=[
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
            ],
            ball_initial_pos=ball_initial_pos,
            play_name=PlayName.FreeKickPlay,
            simulated_test_runner=simulated_test_runner,
        )

    # TODO (#3636): add more validations
    eventually_validations = [[FriendlyTeamEventuallyScored()] if must_score else []]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        test_timeout_s=10,
    )


@pytest.mark.parametrize(
    "ball_initial_pos,yellow_bots",
    [
        # not close to our net
        (
            tbots_cpp.Point(0.9, 2.85),
            [
                tbots_cpp.Point(1, 3),
                tbots_cpp.Point(-2, -1.25),
                tbots_cpp.Point(-1, -0.25),
                tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXNegYCorner(),
                tbots_cpp.Field.createSSLDivisionBField()
                .enemyDefenseArea()
                .negXPosYCorner(),
            ],
        ),
        # close to our net
        (
            tbots_cpp.Point(-2.4, 1),
            [
                tbots_cpp.Point(-2.3, 1.05),
                tbots_cpp.Point(-3.5, 2),
                tbots_cpp.Point(-1.2, 0),
                tbots_cpp.Point(-2.3, -1),
                tbots_cpp.Point(-3.8, -2),
                tbots_cpp.Field.createSSLDivisionBField().enemyGoalCenter(),
            ],
        ),
    ],
)
def test_free_kick_play_enemy(ball_initial_pos, yellow_bots, simulated_test_runner):
    def setup(*args):
        free_kick_play_setup(
            blue_bots=[
                tbots_cpp.Point(-4.5, 0),
                tbots_cpp.Point(-3, 1.5),
                tbots_cpp.Point(-3, 0.5),
                tbots_cpp.Point(-3, -0.5),
                tbots_cpp.Point(-3, -1.5),
                tbots_cpp.Point(4, -2.5),
            ],
            yellow_bots=yellow_bots,
            ball_initial_pos=ball_initial_pos,
            play_name=PlayName.EnemyFreeKickPlay,
            simulated_test_runner=simulated_test_runner,
        )

    # TODO (#3636): add validations
    simulated_test_runner.run_test(
        setup=setup,
        test_timeout_s=10,
    )


@pytest.mark.parametrize(
    "ball_initial_pos",
    [
        tbots_cpp.Point(1.5, -2.75),
        tbots_cpp.Point(-1.5, -2.75),
        tbots_cpp.Point(1.5, -3),
    ],
)
def test_free_kick_play_both(ball_initial_pos, simulated_test_runner):
    def setup(*args):
        free_kick_play_setup(
            blue_bots=[
                tbots_cpp.Point(-3, 0.25),
                tbots_cpp.Point(-3, 1.5),
                tbots_cpp.Point(-3, 0.5),
                tbots_cpp.Point(-3, -0.5),
                tbots_cpp.Point(-3, -1.5),
                tbots_cpp.Point(-3, -0.25),
            ],
            yellow_bots=[
                tbots_cpp.Point(3, 0.25),
                tbots_cpp.Point(3, 1.5),
                tbots_cpp.Point(3, 0.5),
                tbots_cpp.Point(3, -0.5),
                tbots_cpp.Point(3, -1.5),
                tbots_cpp.Point(3, -0.25),
            ],
            ball_initial_pos=ball_initial_pos,
            play_name=PlayName.EnemyFreeKickPlay,
            simulated_test_runner=simulated_test_runner,
        )

    # TODO (#3636): add validations
    simulated_test_runner.run_test(
        setup=setup,
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
