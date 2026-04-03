import pytest
import software.python_bindings as tbots_cpp

from proto.import_all_protos import *

from proto.play_pb2 import PlayName
from proto.ssl_gc_common_pb2 import Team

from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.validation.friendly_team_scored import (
    FriendlyTeamEventuallyScored,
)
from software.simulated_tests.validation.robot_enters_region import RobotEventuallyEntersRegion
from software.simulated_tests.validation.ball_kicked_in_direction import BallEventuallyKickedInDirection
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)


@pytest.mark.parametrize(
    "ball_initial_pos,must_score",
    [
        (tbots_cpp.Point(1.5, -2.75), False),
        (tbots_cpp.Point(-1.5, -2.75), False),
        (tbots_cpp.Point(1.5, -3), False),
        (tbots_cpp.Point(1.5, -0.5), True),
        (tbots_cpp.Point(1.5, 0.5), True),
    ],
)
def test_free_kick_play_friendly(ball_initial_pos, must_score, simulated_test_runner):
    def setup(*args):
        blue_bots = [
            tbots_cpp.Point(-4.5, 0),
            tbots_cpp.Point(-3, 1.5),
            tbots_cpp.Point(-3, 0.5),
            tbots_cpp.Point(-3, -0.5),
            tbots_cpp.Point(-3, -1.5),
            tbots_cpp.Point(4, -2.5),
        ]
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

        simulated_test_runner.set_plays(blue_play=PlayName.FreeKickPlay, yellow_play=PlayName.HaltPlay)

    eventually_validations = [[FriendlyTeamEventuallyScored()] if must_score else [], [
        RobotEventuallyEntersRegion(regions=[tbots_cpp.Circle(ball_initial_pos, 0.1)]),
        # Gets kicked in any direction
        BallEventuallyKickedInDirection(tbots_cpp.Angle.zero(), max_angle_difference_degrees=360)
    ]]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        test_timeout_s=10,
    )


if __name__ == "__main__":
    pytest_main(__file__)
