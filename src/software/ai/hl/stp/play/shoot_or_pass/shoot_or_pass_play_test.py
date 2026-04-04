import pytest

import software.python_bindings as tbots_cpp
from proto.import_all_protos import *
from proto.play_pb2 import PlayName
from proto.ssl_gc_common_pb2 import Team
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.validation.friendly_team_scored import (
    FriendlyTeamEventuallyScored,
)
from software.simulated_tests.simulated_test_fixture import pytest_main


@pytest.mark.skip(
    "Skipping test. TODO (#3233): attacker robot sometimes doesn't kick the ball towards the receiver"
)
def test_shoot_or_pass_play(simulated_test_runner):
    field = tbots_cpp.Field.createSSLDivisionBField()

    def setup(*args):
        simulated_test_runner.set_world_state(
            create_world_state(
                blue_robot_locations=[
                    field.friendlyGoalCenter(),
                    tbots_cpp.Point(-4.5, 3.0),
                    tbots_cpp.Point(-2, 1.5),
                    tbots_cpp.Point(-2, 0.5),
                    tbots_cpp.Point(-2, -1.7),
                    tbots_cpp.Point(-2, -1.5),
                ],
                yellow_robot_locations=[
                    tbots_cpp.Point(1, 0),
                    tbots_cpp.Point(1, 2.5),
                    tbots_cpp.Point(1, -2.5),
                    field.enemyGoalCenter(),
                    field.enemyDefenseArea().negXNegYCorner(),
                    field.enemyDefenseArea().negXPosYCorner(),
                ],
                ball_location=tbots_cpp.Point(-4.4, 2.9),
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.send_gamecontroller_command(
            gc_command=Command.Type.FORCE_START, team=Team.BLUE
        )

        simulated_test_runner.set_plays(
            blue_play=PlayName.ShootOrPassPlay, yellow_play=PlayName.HaltPlay
        )

    # Eventually Validation
    eventually_validations = [[FriendlyTeamEventuallyScored()]]

    simulated_test_runner.run_test(
        setup=setup,
        inv_eventually_validation_sequence_set=eventually_validations,
        ag_eventually_validation_sequence_set=eventually_validations,
        test_timeout_s=15,
        run_till_end=False,
    )


if __name__ == "__main__":
    pytest_main(__file__)
