from typing import override

import software.python_bindings as tbots_cpp
from proto.play_pb2 import Play, PlayName
from proto.ssl_gc_common_pb2 import Team
from software.py_constants import ROBOT_MAX_RADIUS_METERS

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.robot_speed_threshold import (
    RobotSpeedEventuallyBelowThreshold,
)
from software.simulated_tests.simulated_test_fixture import (
    pytest_main,
)
from software.simulated_tests.validation import (
    Validation,
    ValidationStatus,
    ValidationType,
    create_validation_geometry,
)


class FriendlyRobotPositionsVisualization(Validation):
    """Always-passing validation that draws circles at each friendly robot position.
    Add to inv_always_validation_sequence_set and run with --enable_thunderscope to
    visualize robot positions during the test.
    """

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        return ValidationStatus.PASSING

    @override
    def get_validation_type(self, world) -> ValidationType:
        return ValidationType.ALWAYS

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        return create_validation_geometry(
            [
                tbots_cpp.Circle(
                    tbots_cpp.Point(
                        robot.current_state.global_position.x_meters,
                        robot.current_state.global_position.y_meters,
                    ),
                    ROBOT_MAX_RADIUS_METERS + 0.05,
                )
                for robot in world.friendly_team.team_robots
            ]
        )

    def __repr__(self):
        return "Friendly robot positions (visualization only)"


def test_stop_play(simulated_test_runner):
    """Test stop play: robots slow down and avoid the ball (STOP referee state)."""

    def setup(*args):
        # Ball at centre (matches C++ test_stop_play_ball_at_centre_robots_spread_out)
        ball_initial_pos = tbots_cpp.Point(0, 0)

        field = tbots_cpp.Field.createSSLDivisionBField()

        blue_bots = [
            tbots_cpp.Point(-4, 0),
            tbots_cpp.Point(-0.3, 0),
            tbots_cpp.Point(0.3, 0),
            tbots_cpp.Point(0, 0.3),
            tbots_cpp.Point(-3, -1.5),
            tbots_cpp.Point(4.6, -3.1),
        ]

        yellow_bots = [
            tbots_cpp.Point(1, 0),
            tbots_cpp.Point(1, 2.5),
            tbots_cpp.Point(1, -2.5),
            field.enemyGoalCenter(),
            field.enemyDefenseArea().negXNegYCorner(),
            field.enemyDefenseArea().negXPosYCorner(),
        ]

        # Game controller: STOP (both teams) - stop play behaviour
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.STOP, team=Team.UNKNOWN
        )
        simulated_test_runner.gamecontroller.send_gc_command(
            gc_command=Command.Type.FORCE_START, team=Team.UNKNOWN
        )

        # Force play override: blue runs StopPlay, yellow runs HaltPlay
        blue_play = Play()
        blue_play.name = PlayName.StopPlay

        yellow_play = Play()
        yellow_play.name = PlayName.HaltPlay

        simulated_test_runner.blue_full_system_proto_unix_io.send_proto(Play, blue_play)
        simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
            Play, yellow_play
        )

        simulated_test_runner.simulator_proto_unix_io.send_proto(
            WorldState,
            create_world_state(
                yellow_robot_locations=yellow_bots,
                blue_robot_locations=blue_bots,
                ball_location=ball_initial_pos,
                ball_velocity=tbots_cpp.Vector(0, 0),
            ),
        )

    # C++ test waits 8s before checking; use 15s timeout so robots have time to slow.
    # Threshold 1.4 m/s: expect robots to eventually slow below the 1.5 m/s STOP limit.
    # TODO (#3658): add an eventually-validation that friendly robots stay at least 0.5 m away
    # from the ball once pytest fixtures support delaying validations (to mirror the
    # C++ robotsAvoidBall(0.5, ...) check).
    simulated_test_runner.run_test(
        setup=setup,
        params=[0],
        inv_always_validation_sequence_set=[
            [FriendlyRobotPositionsVisualization()],
        ],
        inv_eventually_validation_sequence_set=[
            [RobotSpeedEventuallyBelowThreshold(1.4)]
        ],
        test_timeout_s=15,
    )


if __name__ == "__main__":
    pytest_main(__file__)
