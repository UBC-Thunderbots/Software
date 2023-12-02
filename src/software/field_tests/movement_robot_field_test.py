import pytest

import software.python_bindings as tbots_cpp
import sys
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *

from software.simulated_tests.simulated_test_fixture import *
from software.logger.logger import createLogger
from software.simulated_tests.robot_enters_region import RobotEventuallyEntersRegion
from proto.message_translation.tbots_protobuf import create_world_state
import math

logger = createLogger(__name__)


# TODO 2908: Support running this test in both simulator or field mode
# this test can be run either in simulation or on the field
# @pytest.mark.parametrize(
#     "robot_x_destination, robot_y_destination",
#     [(-2.0, -1), (-2.0, 1.0), (0.0, 1.0), (0.0, -1.0)],
# )
# def test_basic_movement(simulated_test_runner):
#
#     robot_starting_x = 0
#     robot_starting_y = 0
#     ROBOT_ID = 0
#     angle = 0
#     robot_x_destination = -2
#     robot_y_destination = -1
#     rob_pos_p = Point(x_meters=robot_x_destination, y_meters=robot_y_destination)
#
#     move_tactic = MoveTactic()
#     move_tactic.destination.CopyFrom(rob_pos_p)
#     move_tactic.final_speed = 0.0
#     move_tactic.dribbler_mode = DribblerMode.OFF
#     move_tactic.final_orientation.CopyFrom(Angle(radians=angle))
#     move_tactic.ball_collision_type = BallCollisionType.AVOID
#     move_tactic.auto_chip_or_kick.CopyFrom(AutoChipOrKick(autokick_speed_m_per_s=0.0))
#     move_tactic.max_allowed_speed_mode = MaxAllowedSpeedMode.PHYSICAL_LIMIT
#     move_tactic.target_spin_rev_per_s = 0.0
#
#     # setup world state
#     initial_worldstate = create_world_state(
#         yellow_robot_locations=[],
#         blue_robot_locations=[tbots_cpp.Point(robot_starting_x, robot_starting_y)],
#         ball_location=tbots_cpp.Point(1, 1),
#         ball_velocity=tbots_cpp.Point(0, 0),
#     )
#     simulated_test_runner.set_worldState(initial_worldstate)
#
#     # Setup Tactic
#     params = AssignedTacticPlayControlParams()
#
#     params.assigned_tactics[ROBOT_ID].move.CopyFrom(move_tactic)
#
#     # Eventually Validation
#     eventually_validation_sequence_set = [
#         [
#             RobotEventuallyEntersRegion(
#                 regions=[
#                     tbots_cpp.Circle(
#                         tbots_cpp.Point(robot_x_destination, robot_y_destination), 0.2
#                     )
#                 ]
#             ),
#         ]
#     ]
#     simulated_test_runner.set_tactics(params, True)
#
#     simulated_test_runner.run_test(
#         eventually_validation_sequence_set=eventually_validation_sequence_set,
#         test_timeout_s=5,
#     )


# this test can only be run on the field
# def test_basic_rotation(field_test_runner):
#     world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
#     if len(world.friendly_team.team_robots) == 0:
#         raise Exception("The first world received had no robots in it!")
#
#     print("Here are the robots:")
#     print([robot.current_state.global_position for robot in world.friendly_team.team_robots])
#
#     id = world.friendly_team.team_robots[0].id
#     print(f"Running test on robot {id}")
#
#     rob_pos_p = robot.current_state.global_position
#     logger.info("staying in pos {rob_pos_p}")
#
#     for angle in test_angles:
#         move_tactic = MoveTactic()
#         move_tactic.destination.CopyFrom(rob_pos_p)
#         move_tactic.final_speed = 0.0
#         move_tactic.dribbler_mode = DribblerMode.OFF
#         move_tactic.final_orientation.CopyFrom(Angle(radians=angle))
#         move_tactic.ball_collision_type = BallCollisionType.AVOID
#         move_tactic.auto_chip_or_kick.CopyFrom(
#             AutoChipOrKick(autokick_speed_m_per_s=0.0)
#         )
#         move_tactic.max_allowed_speed_mode = MaxAllowedSpeedMode.PHYSICAL_LIMIT
#         move_tactic.target_spin_rev_per_s = 0.0
#
#         # Setup Tactic
#         params = AssignedTacticPlayControlParams()
#
#         params.assigned_tactics[id].move.CopyFrom(move_tactic)
#
#         field_test_runner.set_tactics(params, True)
#         field_test_runner.run_test(
#             always_validation_sequence_set=[[]],
#             eventually_validation_sequence_set=[[]],
#             test_timeout_s=5,
#         )
#         # Send a stop tactic after the test finishes
#         stop_tactic = StopTactic()
#         params = AssignedTacticPlayControlParams()
#         params.assigned_tactics[id].stop.CopyFrom(stop_tactic)
#         # send the stop tactic
#         field_test_runner.set_tactics(params, True)
#
#         # validate by eye
#         logger.info(f"robot set to {angle} orientation")
#
#         time.sleep(2)


def test_one_robots_square(field_test_runner):
    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    if len(world.friendly_team.team_robots) == 0:
        raise Exception("The first world received had no robots in it!")

    print("Here are the robots:")
    print(
        [
            robot.current_state.global_position
            for robot in world.friendly_team.team_robots
        ]
    )

    id = world.friendly_team.team_robots[0].id
    print(f"Running test on robot {id}")

    point1 = Point(x_meters=-0.3, y_meters=0.6)
    point2 = Point(x_meters=-0.3, y_meters=-0.6)
    point3 = Point(x_meters=-1.5, y_meters=-0.6)
    point4 = Point(x_meters=-1.5, y_meters=0.6)

    tactic_0 = MoveTactic(
        destination=point1,
        final_speed=0.0,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        target_spin_rev_per_s=0.0,
    )
    tactic_1 = MoveTactic(
        destination=point2,
        final_speed=0.0,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        target_spin_rev_per_s=0.0,
    )
    tactic_2 = MoveTactic(
        destination=point3,
        final_speed=0.0,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        target_spin_rev_per_s=0.0,
    )
    tactic_3 = MoveTactic(
        destination=point4,
        final_speed=0.0,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=-math.pi / 2),
        ball_collision_type=BallCollisionType.AVOID,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        target_spin_rev_per_s=0.0,
    )
    tactics = [tactic_0, tactic_1, tactic_2, tactic_3]

    for tactic in tactics:
        print(f"Going to {tactic.destination}")
        params = AssignedTacticPlayControlParams()
        params.assigned_tactics[id].move.CopyFrom(tactic)

        field_test_runner.set_tactics(params, True)
        field_test_runner.run_test(
            always_validation_sequence_set=[[]],
            eventually_validation_sequence_set=[[]],
            test_timeout_s=4,
        )

    # Send a stop tactic after the test finishes
    stop_tactic = StopTactic()
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[id].stop.CopyFrom(stop_tactic)


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
