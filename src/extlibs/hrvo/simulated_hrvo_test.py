import pytest
from enum import Enum
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from software.simulated_tests.robot_enters_region import *
import software.python_bindings as tbots
from proto.ssl_gc_common_pb2 import Team
from proto.message_translation.tbots_protobuf import create_world_state_from_state


class HrvoTestVariants(Enum):
    MOVING_ENEMY_BEHIND = 0
    MOVING_ENEMY_SIDE = 1
    NO_OBSTACLE = 2
    FRIENDLY_ROBOT_IN_FRONT = 3
    SINGLE_ENEMY_IN_FRONT = 4
    THREE_ROBOT_WALL = 5
    ZIG_ZAG = 6
    NOT_GOING_IN_STATIC_OBSTACLES = 7
    LOCAL_MINIMA = 8
    LOCAL_MINIMA_OPEN_END = 9
    AVOIDING_BALL_OBSTACLE = 10
    DRIVE_8_STRAIGHT_CIRCLE = 11


def get_zig_zag_params(front_wall_x, robot_y_delta, num_walls):
    return (
        [Point(x_meters=front_wall_x - 0.5, y_meters=0)],
        [Point(x_meters=front_wall_x + float(3) / num_walls + 0.5, y_meters=0)],
        [
            RobotState(global_position=Point(x_meters=x_meters, y_meters=float(y_meters) / 10))
            for y_meters in range(0, 3 * int(robot_y_delta * 10), int(robot_y_delta * 10))
            for x_meters in range(front_wall_x, front_wall_x + num_walls + 1, 1)
        ],
        None, None
    )


def get_robot_circle_params(radius, num_robots):
    return (
        [
            Point(
                x_meters=radius * math.cos(i * 2 * math.pi / num_robots),
                y_meters=radius * math.sin(i * 2 * math.pi / num_robots)
            )
            for i in range(num_robots)
        ],
        [
            Point(
                x_meters=-radius * math.cos(i * 2 * math.pi / num_robots),
                y_meters=-radius * math.sin(i * 2 * math.pi / num_robots)
            )
            for i in range(num_robots)
        ],
        [], None, None
    )


def hrvo_test_setup(
        friendly_robots_positions,
        friendly_robots_destinations,
        enemy_robots,
        ball_initial_pos,
        ball_initial_vel,
        simulated_test_runner
):
    desired_orientation = Angle(radians=0)

    ball_initial_pos = (
        ball_initial_pos
        if ball_initial_pos
        else tbots.Point(1, 2)
    )

    ball_initial_vel = (
        ball_initial_vel
        if ball_initial_vel
        else tbots.Point(0, 0)
    )

    friendly_robots = [
        RobotState(global_position=robot_position)
        for robot_position in friendly_robots_positions
    ]

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    params = AssignedTacticPlayControlParams()

    for index, destination in enumerate(friendly_robots_destinations):
        params = get_move_update_control_params(
            index,
            destination,
            Angle(radians=0),
            0,
            params=params
        )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup no tactics on the enemy side
    params = AssignedTacticPlayControlParams()
    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, params
    )

    # Setup Robots
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state_from_state(
            yellow_robot_states=enemy_robots,
            blue_robot_states=friendly_robots,
            ball_location=ball_initial_pos,
            ball_velocity=ball_initial_vel,
        ),
    )

# TODO: add tests
#   - enemy collision
#   - friendly collision
#   - enemy and friendly moving to places
# TODO: add always collision validation
# TODO: print message for if robot is not exactly stationary
@pytest.mark.parametrize(
    "friendly_robot_positions,friendly_robot_destinations,enemy_robots,ball_initial_pos,ball_initial_vel",
    [
        # robot moving straight with a moving enemy robot behind it
        (
            [Point(x_meters=-2.3, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [RobotState(
                global_position=Point(x_meters=-4.2, y_meters=0),
                global_velocity=Vector(x_component_meters=6, y_component_meters=0),
            )],
            None, None
        ),
        # robot moving straight with a moving enemy robot to its side
        (
            [Point(x_meters=-2.5, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [RobotState(
                global_position=Point(x_meters=-1, y_meters=-8),
                global_velocity=Vector(x_component_meters=0, y_component_meters=5.9),
            )],
            None, None
        ),
        # robot moving straight with no obstacles
        (
            [Point(x_meters=-2.5, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [],
            None, None
        ),
        # robot moving with a stationary friendly robot in front of it
        (
            [Point(x_meters=-2.5, y_meters=0), Point(x_meters=2, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0), Point(x_meters=2, y_meters=0)],
            [],
            None, None
        ),
        # robot moving with a stationary enemy robot in front of it
        (
            [Point(x_meters=0.7, y_meters=0)],
            [Point(x_meters=2, y_meters=0)],
            [RobotState(global_position=Point(x_meters=1, y_meters=0))],
            None, None
        ),
        # robot moving with a 3 enemy robot wall in front of it
        (
            [Point(x_meters=0, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [
                RobotState(global_position=Point(x_meters=1, y_meters=0)),
                RobotState(global_position=Point(x_meters=1, y_meters=0.3)),
                RobotState(global_position=Point(x_meters=1, y_meters=-0.3)),
            ],
            None, None
        ),
        # robot moving with various stationary enemy robots in front of it
        # walls generated by range(start_x, start_x + step * num_robots, step)
        (
            [Point(x_meters=2.9, y_meters=-1)],
            [Point(x_meters=2.9, y_meters=1)],
            [
                RobotState(global_position=Point(x_meters=float(x_meters) / 10, y_meters=0))
                for x_meters in range(20, 32, 2)
            ],
            None, None
        ),
        # robot moving in a local minima (enemy robots in a curve around it)
        (
                [Point(x_meters=0.7, y_meters=0)],
                [Point(x_meters=2.8, y_meters=1)],
                [
                    RobotState(global_position=Point(x_meters=1, y_meters=0)),
                    RobotState(global_position=Point(x_meters=1, y_meters=0.3)),
                    RobotState(global_position=Point(x_meters=1, y_meters=0.6)),
                    RobotState(global_position=Point(x_meters=0.7, y_meters=0.6)),
                    RobotState(global_position=Point(x_meters=1, y_meters=-0.3)),
                    RobotState(global_position=Point(x_meters=1, y_meters=-0.6)),
                    RobotState(global_position=Point(x_meters=0.7, y_meters=-0.6)),
                ],
                None, None
        ),
        # robot moving in a local minima (enemy robots in a curve around it) with an opening in the middle
        (
                [Point(x_meters=0.7, y_meters=0)],
                [Point(x_meters=2.8, y_meters=1)],
                [
                    RobotState(global_position=Point(x_meters=1.5, y_meters=0)),
                    RobotState(global_position=Point(x_meters=1, y_meters=0.3)),
                    RobotState(global_position=Point(x_meters=1, y_meters=0.6)),
                    RobotState(global_position=Point(x_meters=0.7, y_meters=0.6)),
                    RobotState(global_position=Point(x_meters=1, y_meters=-0.3)),
                    RobotState(global_position=Point(x_meters=1, y_meters=-0.6)),
                    RobotState(global_position=Point(x_meters=0.7, y_meters=-0.6)),
                ],
                None, None
        ),
        # robot moving in a zig zag path around enemy robots
        get_zig_zag_params(-2, 0.2, 3),
        # robots in a circle moving along each diameter
        get_robot_circle_params(1.5, 8),

    ]
)
def test_robot_movement(
        simulated_test_runner,
        friendly_robot_positions,
        friendly_robot_destinations,
        enemy_robots,
        ball_initial_pos,
        ball_initial_vel
):
    # Always Validation
    always_validation_sequence_set = [[]]

    # Eventually Validation
    eventually_validation_sequence_set = get_reached_destination_validation(friendly_robot_destinations)

    simulated_test_runner.run_test(
        setup=lambda param: hrvo_test_setup(
            friendly_robot_positions,
            friendly_robot_destinations,
            enemy_robots,
            ball_initial_pos,
            ball_initial_vel,
            simulated_test_runner
        ),
        params=[0],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


#
# def test_drive_straight_moving_enemy_behind(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
#         HrvoTestVariants.MOVING_ENEMY_BEHIND.name
#     ]
#
#     destination = setup_param["destination"]
#     threshold = 0.05
#     # Eventually Validation
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=10,
#     )
#
#
# def test_drive_straight_moving_enemy_side(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[HrvoTestVariants.MOVING_ENEMY_SIDE.name]
#
#     destination = setup_param["destination"]
#     threshold = 0.05
#     # Eventually Validation
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=10,
#     )
#
#
# def test_drive_straight_no_obstacle(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[HrvoTestVariants.NO_OBSTACLE.name]
#
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=10,
#     )
#
#
# def test_drive_straight_friendly_in_front(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
#         HrvoTestVariants.FRIENDLY_ROBOT_IN_FRONT.name
#     ]
#
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=10,
#     )
#
#
# def test_single_enemy_directly_in_front(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
#         HrvoTestVariants.SINGLE_ENEMY_IN_FRONT.name
#     ]
#
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=10,
#     )
#
#
# def test_zig_zag(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = get_zig_zag_params()
#
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=10,
#     )
#
#
# def test_not_going_in_static_obstacles(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
#         HrvoTestVariants.NOT_GOING_IN_STATIC_OBSTACLES.name
#     ]
#
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=10,
#     )
#
#
# def test_start_in_local_minima(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[HrvoTestVariants.LOCAL_MINIMA.name]
#
#     eventually_validation_sequence_set = [[]]
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=20,
#     )
#
#
# def test_start_in_local_minima_open_end(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
#         HrvoTestVariants.LOCAL_MINIMA_OPEN_END.name
#     ]
#
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=20,
#     )
#
#
# def test_robot_avoiding_ball_obstacle(simulated_test_runner):
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
#         HrvoTestVariants.AVOIDING_BALL_OBSTACLE.name
#     ]
#
#     eventually_validation_sequence_set = get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
#         params=[setup_param],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=20,
#     )
#
#
# def test_8_driving_straight_circle(simulated_test_runner):
#
#     def setup(setup_param):
#         ball_initial_pos = tbots.Point(1, 2)
#         ball_initial_vel = tbots.Point(0, 0)
#
#         radius, num_robots = setup_param
#
#         friendly_robots = []
#
#         # Game Controller Setup
#         simulated_test_runner.gamecontroller.send_ci_input(
#             gc_command=Command.Type.STOP, team=Team.UNKNOWN
#         )
#         simulated_test_runner.gamecontroller.send_ci_input(
#             gc_command=Command.Type.FORCE_START, team=Team.BLUE
#         )
#
#         params = AssignedTacticPlayControlParams()
#
#         for i in range(num_robots):
#             angle = i * 2 * math.pi / num_robots
#             x_meters = radius * math.cos(angle)
#             y_meters = radius * math.sin(angle)
#
#             friendly_robots.append(RobotState(
#                 global_position=Point(
#                     x_meters=x_meters,
#                     y_meters=y_meters
#                 )
#             ))
#
#             params = get_move_update_control_params(
#                 i, Point(x_meters=-x_meters, y_meters=-y_meters),
#                 Angle(radians=angle + math.pi), 0,
#                 params=params
#             )
#
#         simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
#             AssignedTacticPlayControlParams, params
#         )
#
#         # Setup Robots
#         simulated_test_runner.simulator_proto_unix_io.send_proto(
#             WorldState,
#             create_world_state_from_state(
#                 yellow_robot_states=[],
#                 blue_robot_states=friendly_robots,
#                 ball_location=ball_initial_pos,
#                 ball_velocity=ball_initial_vel,
#             ),
#         )
#
#     # Always Validation
#     always_validation_sequence_set = [[]]
#
#     eventually_validation_sequence_set = [[]]
#
#     #get_reached_destination_validation(setup_param)
#
#     simulated_test_runner.run_test(
#         setup=setup,
#         params=[(1.5, 4)],
#         inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
#         inv_always_validation_sequence_set=always_validation_sequence_set,
#         ag_eventually_validation_sequence_set=[[]],
#         ag_always_validation_sequence_set=[[]],
#         test_timeout_s=20,
#     )


def get_reached_destination_validation(robot_destinations):
    threshold = 0.05
    # Eventually Validation
    return [
        [
            RobotEventuallyEntersRegionAndStops(
                robot_id,
                regions=[
                    tbots.Rectangle(
                        tbots.Point(
                            destination.x_meters - threshold,
                            destination.y_meters - threshold,
                        ),
                        tbots.Point(
                            destination.x_meters + threshold,
                            destination.y_meters + threshold,
                        ),
                    )
                ],
                num_ticks=15,
            )
        ]
        for robot_id, destination in enumerate(robot_destinations)
    ]


def get_move_update_control_params(
    robot_id, destination, desired_orientation, final_speed, params=None
):
    params = params if params else AssignedTacticPlayControlParams()
    params.assigned_tactics[robot_id].move.CopyFrom(
        MoveTactic(
            destination=destination,
            final_orientation=desired_orientation,
            final_speed=final_speed,
            dribbler_mode=DribblerMode.OFF,
            ball_collision_type=BallCollisionType.ALLOW,
            auto_chip_or_kick=AutoChipOrKick(autokick_speed_m_per_s=0.0),
            max_allowed_speed_mode=MaxAllowedSpeedMode.PHYSICAL_LIMIT,
            target_spin_rev_per_s=0.0,
        )
    )

    return params


get_zig_zag_params.__test__ = False
get_move_update_control_params.__test__ = False
get_reached_destination_validation.__test__ = False
hrvo_test_setup.__test__ = False

if __name__ == "__main__":
    pytest_main(__file__)
