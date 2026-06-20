import math

import software.python_bindings as tbots_cpp

from proto.import_all_protos import *
from proto.message_translation.tbots_protobuf import create_world_state
from software.simulated_tests.simulated_test_fixture import pytest_main


def test_one_robot_square(simulated_test_runner):
    robot_id = 0
    corners = [
        tbots_cpp.Point(-0.3, 0.6),
        tbots_cpp.Point(-0.3, -0.6),
        tbots_cpp.Point(-1.5, -0.6),
        tbots_cpp.Point(-1.5, 0.6),
    ]

    start_positions = [tbots_cpp.Point(0, 0)] + corners[:-1]

    for start, destination in zip(start_positions, corners):

        def setup(*args, s=start, d=destination):
            simulated_test_runner.set_world_state(
                create_world_state(
                    blue_robot_locations=[s],
                    yellow_robot_locations=[],
                    ball_location=tbots_cpp.Point(4, 0),
                    ball_velocity=tbots_cpp.Vector(0, 0),
                )
            )
            simulated_test_runner.set_tactics(
                blue_tactics={
                    robot_id: MoveTactic(
                        destination=tbots_cpp.createPointProto(d),
                        dribbler_mode=DribblerMode.OFF,
                        final_orientation=tbots_cpp.createAngleProto(
                            tbots_cpp.Angle.fromRadians(-math.pi / 2)
                        ),
                        ball_collision_type=BallCollisionType.AVOID,
                        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
                        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
                        obstacle_avoidance_mode=ObstacleAvoidanceMode.SAFE,
                    )
                }
            )

        simulated_test_runner.run_test(
            setup=setup,
            inv_eventually_validation_sequence_set=[[]],
            ag_eventually_validation_sequence_set=[[]],
            test_timeout_s=4,
        )

    simulated_test_runner.set_tactics(
        blue_tactics={robot_id: HaltTactic()}, yellow_tactics=None
    )


if __name__ == "__main__":
    pytest_main(__file__)
