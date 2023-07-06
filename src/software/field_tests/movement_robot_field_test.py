import pytest

import software.python_bindings as tbots
import sys
from proto.ssl_gc_common_pb2 import Team
from proto.import_all_protos import *
from software.field_tests.field_test_fixture import *

from software.simulated_tests.simulated_test_fixture import *
from software.logger.logger import createLogger
from software.simulated_tests.robot_enters_region import RobotEventuallyEntersRegion
from proto.message_translation.tbots_protobuf import create_world_state

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
#         blue_robot_locations=[tbots.Point(robot_starting_x, robot_starting_y)],
#         ball_location=tbots.Point(1, 1),
#         ball_velocity=tbots.Point(0, 0),
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
#                     tbots.Circle(
#                         tbots.Point(robot_x_destination, robot_y_destination), 0.2
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
#     test_angles = [0, 45, 90, 180, 270, 0]
#     id = 5
#
#     # current position
#     world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
#     robot = [robot for robot in world.friendly_team.team_robots if robot.id == id][0]
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



# def test_pivot_kick(field_test_runner):
#     id = 5
#
#     world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
#     print("Here are the robots:")
#     print([robot.current_state.global_position for robot in world.friendly_team.team_robots])
#
#     params = AssignedTacticPlayControlParams()
#     params.assigned_tactics[id].pivot_kick.CopyFrom(
#         PivotKickTactic(
#             kick_origin = Point(x_meters=-1.13, y_meters=0.75),
#             kick_direction = Angle(radians=-math.pi/2),
#             auto_chip_or_kick = AutoChipOrKick(autokick_speed_m_per_s=5.0)
#         )
#     )
#
#     field_test_runner.set_tactics(params, True)
#     field_test_runner.run_test(
#         always_validation_sequence_set=[[]],
#         eventually_validation_sequence_set=[[]],
#         test_timeout_s=20,
#     )
#     # Send a stop tactic after the test finishes
#     stop_tactic = StopTactic()
#     params = AssignedTacticPlayControlParams()
#     params.assigned_tactics[id].stop.CopyFrom(stop_tactic)

import math

def test_one_robots_foward_back(field_test_runner):
    id1 = 1

    world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
    print("Here are the robots:")
    print([robot.current_state.global_position for robot in world.friendly_team.team_robots])

    tactic_pos_y = MoveTactic(
        destination=Point(x_meters=1.5, y_meters=2.5),
        final_speed=0.0,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=math.pi),
        ball_collision_type=BallCollisionType.ALLOW,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        target_spin_rev_per_s=0.0
    )
    tactic_neg_y = MoveTactic(
        destination=Point(x_meters=1.5, y_meters=1.5),
        final_speed=0.0,
        dribbler_mode=DribblerMode.OFF,
        final_orientation=Angle(radians=0),
        ball_collision_type=BallCollisionType.ALLOW,
        auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
        max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
        target_spin_rev_per_s=0.0
    )

    for test_id in range(12):
        params = AssignedTacticPlayControlParams()
        params.assigned_tactics[id1].move.CopyFrom(tactic_neg_y if test_id % 2 == 0 else tactic_pos_y)

        field_test_runner.set_tactics(params, True)
        field_test_runner.run_test(
            always_validation_sequence_set=[[]],
            eventually_validation_sequence_set=[[]],
            test_timeout_s=0.8,
        )

    # Send a stop tactic after the test finishes
    stop_tactic = StopTactic()
    params = AssignedTacticPlayControlParams()
    params.assigned_tactics[id1].stop.CopyFrom(stop_tactic)


# def test_one_robots_square(field_test_runner):
#     id1 = 5
#
#     world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
#     print("Here are the robots:")
#     print([robot.current_state.global_position for robot in world.friendly_team.team_robots])
#
#     tactic_0 = MoveTactic(
#         destination=Point(x_meters=-0.3, y_meters=2.0),
#         final_speed=0.0,
#         dribbler_mode=DribblerMode.OFF,
#         final_orientation=Angle(radians=-math.pi/2),
#         ball_collision_type=BallCollisionType.AVOID,
#         auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
#         max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
#         target_spin_rev_per_s=0.0
#     )
#     tactic_1 = MoveTactic(
#         destination=Point(x_meters=-0.3, y_meters=-2.0),
#         final_speed=0.0,
#         dribbler_mode=DribblerMode.OFF,
#         final_orientation=Angle(radians=-math.pi/2),
#         ball_collision_type=BallCollisionType.AVOID,
#         auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
#         max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
#         target_spin_rev_per_s=0.0
#     )
#     tactic_2 = MoveTactic(
#         destination=Point(x_meters=-1.3, y_meters=-2.0),
#         final_speed=0.0,
#         dribbler_mode=DribblerMode.OFF,
#         final_orientation=Angle(radians=-math.pi/2),
#         ball_collision_type=BallCollisionType.AVOID,
#         auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
#         max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
#         target_spin_rev_per_s=0.0
#     )
#     tactic_3 = MoveTactic(
#         destination=Point(x_meters=-1.3, y_meters=2.0),
#         final_speed=0.0,
#         dribbler_mode=DribblerMode.OFF,
#         final_orientation=Angle(radians=-math.pi/2),
#         ball_collision_type=BallCollisionType.AVOID,
#         auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
#         max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
#         target_spin_rev_per_s=0.0
#     )
#     tactics = [tactic_0, tactic_1, tactic_2, tactic_3]
#
#     for tactic in tactics:
#         print(f"Going to {tactic.destination}")
#         params = AssignedTacticPlayControlParams()
#         params.assigned_tactics[id1].move.CopyFrom(tactic)
#
#         field_test_runner.set_tactics(params, True)
#         field_test_runner.run_test(
#             always_validation_sequence_set=[[]],
#             eventually_validation_sequence_set=[[]],
#             test_timeout_s=4,
#         )
#
#
#     # Send a stop tactic after the test finishes
#     stop_tactic = StopTactic()
#     params = AssignedTacticPlayControlParams()
#     params.assigned_tactics[id1].stop.CopyFrom(stop_tactic)


# def test_two_robots_obstacle_avoidance(field_test_runner):
#     id1 = 4
#     id2 = 5
#
#     world = field_test_runner.world_buffer.get(block=True, timeout=WORLD_BUFFER_TIMEOUT)
#     print("Here are the robots:")
#     print([robot.current_state.global_position for robot in world.friendly_team.team_robots])
#
#     id1_pos_y = False
#     tactic_pos_y = MoveTactic(
#         destination=Point(x_meters=-1.2, y_meters=0.7),
#         final_speed=0.0,
#         dribbler_mode=DribblerMode.OFF,
#         final_orientation=Angle(radians=-math.pi/2),
#         ball_collision_type=BallCollisionType.AVOID,
#         auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
#         max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
#         target_spin_rev_per_s=0.0
#     )
#     tactic_neg_y = MoveTactic(
#         destination=Point(x_meters=-1.2, y_meters=-0.7),
#         final_speed=0.0,
#         dribbler_mode=DribblerMode.OFF,
#         final_orientation=Angle(radians=-math.pi/2),
#         ball_collision_type=BallCollisionType.AVOID,
#         auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
#         max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
#         target_spin_rev_per_s=0.0
#     )
#
#     for test_id in range(2):
#         params = AssignedTacticPlayControlParams()
#         params.assigned_tactics[id1 if id1_pos_y else id2].move.CopyFrom(tactic_pos_y)
#         params.assigned_tactics[id2 if id1_pos_y else id1].move.CopyFrom(tactic_neg_y)
#         id1_pos_y = not id1_pos_y
#
#         field_test_runner.set_tactics(params, True)
#         field_test_runner.run_test(
#             always_validation_sequence_set=[[]],
#             eventually_validation_sequence_set=[[]],
#             test_timeout_s=6,
#         )
#
#
#     # Send a stop tactic after the test finishes
#     stop_tactic = StopTactic()
#     params = AssignedTacticPlayControlParams()
#     params.assigned_tactics[id1].stop.CopyFrom(stop_tactic)
#     params.assigned_tactics[id2].stop.CopyFrom(stop_tactic)


if __name__ == "__main__":
    # Run the test, -s disables all capturing at -vv increases verbosity
    sys.exit(pytest.main([__file__, "-svv"]))
