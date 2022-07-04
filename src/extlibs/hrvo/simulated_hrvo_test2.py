import pytest

import software.python_bindings as tbots
from software.simulated_tests.excessive_dribbling import *
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.robot_enters_region import *
from software.simulated_tests.robot_speed_threshold import *


@pytest.mark.parametrize(
    "yellow_robot_initial_position_list,yellow_robot_destination_list,blue_robot_initial_position_list,blue_robot_destination_list",
    [
        (
            [
                tbots.Point(-2.8, 1),
                tbots.Point(2.8, 1),
                tbots.Point(-2.8, 2),
                tbots.Point(2.8, 2),
                tbots.Point(0, 2),
            ],
            [
                tbots.Point(2.8, 1),
                tbots.Point(-2.8, 1),
                tbots.Point(2.8, 2),
                tbots.Point(-2.8, 2),
                tbots.Point(0, -2),
            ],
            [
                tbots.Point(-2.8, -1),
                tbots.Point(2.8, -1),
                tbots.Point(-2.8, -2),
                tbots.Point(2.8, -2),
                tbots.Point(0, -2),
            ],
            [
                tbots.Point(2.8, -1),
                tbots.Point(-2.8, -1),
                tbots.Point(2.8, -2),
                tbots.Point(-2.8, -2),
                tbots.Point(0, 2),
            ],
        ),
    ],
)
def test_hrvo_motion_planning(
    yellow_robot_initial_position_list,
    yellow_robot_destination_list,
    blue_robot_initial_position_list,
    blue_robot_destination_list,
    simulated_test_runner,
):
    # Setup Robot
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=yellow_robot_initial_position_list,
            blue_robot_locations=blue_robot_initial_position_list,
            ball_location=tbots.Point(-4, -4),
            ball_velocity=tbots.Vector(0, 0),
        ),
    )

    # Setup Tactic
    params = AssignedTacticPlayControlParams()
    for id, dest in enumerate(blue_robot_destination_list):
        params.assigned_tactics[id].move.CopyFrom(
            MoveTactic(
                destination=tbots.createPointProto(dest),
                final_orientation=Angle(radians=0),
                final_speed=0,
                dribbler_mode=DribblerMode.OFF,
                ball_collision_type=BallCollisionType.ALLOW,
                auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0),
                max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                target_spin_rev_per_s=0,
            )
        )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    for id, dest in enumerate(yellow_robot_destination_list):
        params.assigned_tactics[id].move.CopyFrom(
            MoveTactic(
                destination=tbots.createPointProto(dest),
                final_orientation=Angle(radians=0),
                final_speed=0,
                dribbler_mode=DribblerMode.OFF,
                ball_collision_type=BallCollisionType.ALLOW,
                auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0),
                max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                target_spin_rev_per_s=0,
            )
        )

    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Always Validation
    always_validation_sequence_set = []

    # Eventually Validation
    eventually_validation_sequence_set = [
        [
            RobotEventuallyEntersRegion(
                regions=[tbots.Circle(tbots.Point(2.8, -1), 0.02)]
            ),
            RobotSpeedEventuallyBelowThreshold(speed_threshold=0.01),
        ]
    ]

    simulated_test_runner.run_test(
        eventually_validation_sequence_set=eventually_validation_sequence_set,
        always_validation_sequence_set=always_validation_sequence_set,
        test_timeout_s=8,
    )


if __name__ == "__main__":
    pytest_main(__file__)
