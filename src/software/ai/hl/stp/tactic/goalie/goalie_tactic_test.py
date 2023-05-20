import pytest

import software.python_bindings as tbots
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.ball_enters_region import *
from software.simulated_tests.ball_stops_in_region import *
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.ball_moves_forward import *
from software.simulated_tests.test_constants import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from proto.ssl_gc_common_pb2 import Team


@pytest.mark.parametrize(
    "ball_initial_position,ball_initial_velocity,robot_initial_position,ball_validation_region",
    [
        # test panic ball very fast in straight line
        (tbots.Point(0, 0), tbots.Vector(-5, 0), tbots.Point(-4, 0), FIELD_RIGHT_HALF),
        # test panic ball very_fast in diagonal line
        # TODO (#2609): failing tests when thunderscope is off
        # (
        #     tbots.Point(0, 0),
        #     tbots.Vector(-5.5, 0.25),
        #     DIV_B_FIELD.friendlyGoalCenter()
        #     + tbots.Vector(0, -0.5),
        # ),
        # test ball very fast misses net
        (
            tbots.Point(0, 0),
            tbots.Vector(-5, 1),
            tbots.Point(-4.5, 0),
            tbots.Rectangle(
                tbots.Point(-FIELD_X_LENGTH / 2, FRIENDLY_DEFENSE_AREA.yLength() / 2),
                tbots.Point(FIELD_X_LENGTH / 2, FIELD_Y_LENGTH / 2),
            ),
        ),
        # test slow ball at sharp angle to friendly goal
        # TODO (#2609): failing tests when thunderscope is off
        # ball slow inside friendly defense area
        # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 0)),
        # # ball slow inside friendly defense area
        # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 2)),
        # # ball slow inside friendly defense area
        # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(0, 2)),
        # ball slow inside friendly defense area
        # (tbots.Point(-4, 0.8), tbots.Vector(-0.2, 0), tbots.Point(-4, 0),),
        # ball stationary inside friendly defense area
        (
            tbots.Point(-4, 0.0),
            tbots.Vector(0.0, 0),
            DIV_B_FIELD.friendlyGoalpostPos(),
            FIELD_RIGHT_HALF,
        ),
        # ball stationary inside no-chip rectangle
        (
            DIV_B_FIELD.friendlyGoalCenter() + tbots.Vector(0.1, 0.1),
            tbots.Vector(-0.2, 0),
            tbots.Point(-4, -1),
            FIELD_RIGHT_HALF,
        ),
        # ball fast inside no-chip rectangle but no intersection with goal
        (
            DIV_B_FIELD.friendlyGoalCenter() + tbots.Vector(0.1, 0),
            tbots.Vector(0, -0.5),
            tbots.Point(-3.5, 1),
            FIELD_RIGHT_HALF,
        ),
        # ball moving out from inside defense area
        (
            DIV_B_FIELD.friendlyGoalCenter() + tbots.Vector(0.5, 0),
            tbots.Vector(0.5, 0),
            tbots.Point(-3.5, 0),
            FIELD_RIGHT_HALF,
        ),
        # ball slow inside no-chip rectangle
        (
            DIV_B_FIELD.friendlyGoalCenter() + tbots.Vector(0.1, 0),
            tbots.Vector(0.1, -0.1),
            tbots.Point(-3.5, 1),
            FIELD_RIGHT_HALF,
        ),
        # ball moving into goal from inside defense area
        (
            DIV_B_FIELD.friendlyGoalCenter() + tbots.Vector(0.5, 0),
            tbots.Vector(-0.5, 0),
            tbots.Point(-3.5, 0),
            FIELD_RIGHT_HALF,
        ),
        # ball moving up and out of defense area
        (
            DIV_B_FIELD.friendlyGoalCenter() + tbots.Vector(0.3, 0),
            tbots.Vector(0, 1),
            tbots.Point(-3.5, 0),
            FIELD_RIGHT_HALF,
        ),
        # ball moving down and out goal from defense area
        (
            DIV_B_FIELD.friendlyGoalCenter() + tbots.Vector(0.3, 0),
            tbots.Vector(0, -0.7),
            tbots.Point(-3.5, 0),
            FIELD_RIGHT_HALF,
        ),
    ],
)
def test_goalie_blocks_shot(
    ball_initial_position,
    ball_initial_velocity,
    robot_initial_position,
    ball_validation_region,
    simulated_test_runner,
):

    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            [],
            blue_robot_locations=[robot_initial_position],
            ball_location=ball_initial_position,
            ball_velocity=ball_initial_velocity,
        ),
    )

    # These aren't necessary for this test, but this is just an example
    # of how to send commands to the simulator.
    #
    # NOTE: The gamecontroller responses are automatically handled by
    # the gamecontroller context manager class
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[0].goalie.CopyFrom(
        GoalieTactic(max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT)
    )
    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    threshold = 0.1

    # Always Validation
    always_validation_sequence_set = [
        [
            RobotNeverEntersRegion(regions=[DIV_B_FIELD.enemyDefenseArea()]),
            RobotAlwaysStaysInRegion(
                regions=[
                    tbots.Rectangle(
                        tbots.Point(
                            FRIENDLY_DEFENSE_AREA.negXNegYCorner().x(),
                            FRIENDLY_DEFENSE_AREA.negXNegYCorner().y() - threshold,
                        ),
                        tbots.Point(
                            FRIENDLY_DEFENSE_AREA.posXPosYCorner().x() + threshold,
                            FRIENDLY_DEFENSE_AREA.posXPosYCorner().y() + threshold,
                        ),
                    ),
                    FRIENDLY_GOAL_AREA,
                ]
            ),
            BallNeverEntersRegion(regions=[DIV_B_FIELD.friendlyGoal()]),
            NeverExcessivelyDribbles(),
        ]
    ]

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            # Goalie should be in the defense area
            BallStopsInRegion(regions=[ball_validation_region])
        ],
    ]

    simulated_test_runner.run_test(
        test_timeout_s=20,
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
    )


if __name__ == "__main__":
    pytest_main(__file__)
