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


DRIVE_IN_STRAIGHT_LINE_PARAMS = {
    HrvoTestVariants.MOVING_ENEMY_BEHIND.name: {
        "initial_position": Point(x_meters=-2.3, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-0, y_meters=1))],
        "enemy_robots": [
            RobotState(
                global_position=Point(x_meters=-4.2, y_meters=0),
                global_velocity=Vector(x_component_meters=5, y_component_meters=0),
            )
        ],
        "destination": Point(x_meters=2.8, y_meters=0),
    },
    HrvoTestVariants.MOVING_ENEMY_SIDE.name: {
        "initial_position": Point(x_meters=-2.5, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-3, y_meters=0))],
        "enemy_robots": [
            RobotState(
                global_position=Point(x_meters=-1, y_meters=-3.5),
                global_velocity=Vector(x_component_meters=0, y_component_meters=6),
            )
        ],
        "destination": Point(x_meters=2.8, y_meters=0),
    },
    HrvoTestVariants.NO_OBSTACLE.name: {
        "initial_position": Point(x_meters=-2.5, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-3, y_meters=0))],
        "enemy_robots": [RobotState(global_position=Point(x_meters=-2, y_meters=-2))],
        "destination": Point(x_meters=2.8, y_meters=0),
    },
    HrvoTestVariants.FRIENDLY_ROBOT_IN_FRONT.name: {
        "initial_position": Point(x_meters=-2.5, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=2, y_meters=0))],
        "enemy_robots": [],
        "destination": Point(x_meters=2.8, y_meters=0),
    },
    HrvoTestVariants.SINGLE_ENEMY_IN_FRONT.name: {
        "initial_position": Point(x_meters=0.7, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-3, y_meters=0))],
        "enemy_robots": [RobotState(global_position=Point(x_meters=1, y_meters=0))],
        "destination": Point(x_meters=2, y_meters=0),
    },
    HrvoTestVariants.THREE_ROBOT_WALL.name: {
        "initial_position": Point(x_meters=0, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-3, y_meters=0))],
        "enemy_robots": [
            RobotState(global_position=Point(x_meters=1, y_meters=0)),
            RobotState(global_position=Point(x_meters=1, y_meters=0.3)),
            RobotState(global_position=Point(x_meters=1, y_meters=-0.3)),
        ],
        "destination": Point(x_meters=2.8, y_meters=0),
    },
    HrvoTestVariants.NOT_GOING_IN_STATIC_OBSTACLES.name: {
        "initial_position": Point(x_meters=2.9, y_meters=-1),
        "friendly_robots": [RobotState(global_position=Point(x_meters=2, y_meters=2))],
        "enemy_robots": [
            RobotState(global_position=Point(x_meters=2, y_meters=0)),
            RobotState(global_position=Point(x_meters=2.2, y_meters=0)),
            RobotState(global_position=Point(x_meters=2.4, y_meters=0)),
            RobotState(global_position=Point(x_meters=2.6, y_meters=0)),
            RobotState(global_position=Point(x_meters=2.8, y_meters=0)),
            RobotState(global_position=Point(x_meters=3, y_meters=0)),
        ],
        "destination": Point(x_meters=2.9, y_meters=1),
    },
    HrvoTestVariants.LOCAL_MINIMA.name: {
        "initial_position": Point(x_meters=0.7, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-3, y_meters=0))],
        "enemy_robots": [
            RobotState(global_position=Point(x_meters=1, y_meters=0)),
            RobotState(global_position=Point(x_meters=1, y_meters=0.3)),
            RobotState(global_position=Point(x_meters=1, y_meters=0.6)),
            RobotState(global_position=Point(x_meters=0.7, y_meters=0.6)),
            RobotState(global_position=Point(x_meters=1, y_meters=-0.3)),
            RobotState(global_position=Point(x_meters=1, y_meters=-0.6)),
            RobotState(global_position=Point(x_meters=0.7, y_meters=-0.6)),
        ],
        "destination": Point(x_meters=2.8, y_meters=0),
    },
    HrvoTestVariants.LOCAL_MINIMA_OPEN_END.name: {
        "initial_position": Point(x_meters=0.7, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-3, y_meters=0))],
        "enemy_robots": [
            RobotState(global_position=Point(x_meters=1.5, y_meters=0)),
            RobotState(global_position=Point(x_meters=1, y_meters=0.3)),
            RobotState(global_position=Point(x_meters=1, y_meters=0.6)),
            RobotState(global_position=Point(x_meters=0.7, y_meters=0.6)),
            RobotState(global_position=Point(x_meters=1, y_meters=-0.3)),
            RobotState(global_position=Point(x_meters=1, y_meters=-0.6)),
            RobotState(global_position=Point(x_meters=0.7, y_meters=-0.6)),
        ],
        "destination": Point(x_meters=2.8, y_meters=0),
    },
    HrvoTestVariants.AVOIDING_BALL_OBSTACLE.name: {
        "initial_position": Point(x_meters=1.0, y_meters=0),
        "friendly_robots": [],
        "enemy_robots": [],
        "destination": Point(x_meters=-1.0, y_meters=0),
    },
}


def get_zig_zag_params():
    front_wall_x = -2
    gate_1 = 1
    gate_2 = gate_1 + 2
    gate_3 = gate_2 + 1
    robot_y_delta = 0.2

    return {
        "initial_position": Point(x_meters=front_wall_x - 0.5, y_meters=0),
        "friendly_robots": [RobotState(global_position=Point(x_meters=-3, y_meters=0))],
        "enemy_robots": [
            RobotState(global_position=Point(x_meters=front_wall_x, y_meters=0)),
            RobotState(
                global_position=Point(x_meters=front_wall_x, y_meters=robot_y_delta)
            ),
            RobotState(
                global_position=Point(x_meters=front_wall_x, y_meters=2 * robot_y_delta)
            ),
            RobotState(
                global_position=Point(x_meters=front_wall_x + gate_1, y_meters=0)
            ),
            RobotState(
                global_position=Point(
                    x_meters=front_wall_x + gate_1, y_meters=-robot_y_delta
                )
            ),
            RobotState(
                global_position=Point(
                    x_meters=front_wall_x + gate_1, y_meters=-2 * robot_y_delta
                )
            ),
            RobotState(
                global_position=Point(x_meters=front_wall_x + gate_2, y_meters=0)
            ),
            RobotState(
                global_position=Point(
                    x_meters=front_wall_x + gate_2, y_meters=robot_y_delta
                )
            ),
            RobotState(
                global_position=Point(
                    x_meters=front_wall_x + gate_2, y_meters=2 * robot_y_delta
                )
            ),
            RobotState(
                global_position=Point(x_meters=front_wall_x + gate_3, y_meters=0)
            ),
            RobotState(
                global_position=Point(
                    x_meters=front_wall_x + gate_3, y_meters=-robot_y_delta
                )
            ),
            RobotState(
                global_position=Point(
                    x_meters=front_wall_x + gate_3, y_meters=-2 * robot_y_delta
                )
            ),
        ],
        "destination": Point(x_meters=front_wall_x + gate_3 + 0.5, y_meters=0),
        "ball_initial_pos": tbots.Point(0, -2),
        "ball_initial_vel": tbots.Vector(0, 6),
    }


def hrvo_test_setup(world_state_dict, simulated_test_runner):
    desired_orientation = Angle(radians=0)
    ball_initial_pos = (
        world_state_dict["ball_initial_pos"]
        if "ball_initial_pos" in world_state_dict
        else tbots.Point(1, 2)
    )

    ball_initial_vel = (
        world_state_dict["ball_initial_vel"]
        if "ball_initial_vel" in world_state_dict
        else tbots.Point(0, 0)
    )

    friendly_robots = world_state_dict["friendly_robots"].__add__(
        [RobotState(global_position=world_state_dict["initial_position"])]
    )

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    params = get_move_update_control_params(
        len(friendly_robots) - 1,
        world_state_dict["destination"],
        desired_orientation,
        0,
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
            yellow_robot_states=world_state_dict["enemy_robots"],
            blue_robot_states=friendly_robots,
            ball_location=ball_initial_pos,
            ball_velocity=ball_initial_vel,
        ),
    )


def test_drive_straight_moving_enemy_behind(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
        HrvoTestVariants.MOVING_ENEMY_BEHIND.name
    ]

    destination = setup_param["destination"]
    threshold = 0.05
    # Eventually Validation
    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_drive_straight_moving_enemy_side(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[HrvoTestVariants.MOVING_ENEMY_SIDE.name]

    destination = setup_param["destination"]
    threshold = 0.05
    # Eventually Validation
    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_drive_straight_no_obstacle(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[HrvoTestVariants.NO_OBSTACLE.name]

    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_drive_straight_friendly_in_front(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
        HrvoTestVariants.FRIENDLY_ROBOT_IN_FRONT.name
    ]

    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_single_enemy_directly_in_front(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
        HrvoTestVariants.SINGLE_ENEMY_IN_FRONT.name
    ]

    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_zig_zag(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = get_zig_zag_params()

    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_not_going_in_static_obstacles(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
        HrvoTestVariants.NOT_GOING_IN_STATIC_OBSTACLES.name
    ]

    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=10,
    )


def test_start_in_local_minima(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[HrvoTestVariants.LOCAL_MINIMA.name]

    eventually_validation_sequence_set = [[]]

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=20,
    )


def test_start_in_local_minima_open_end(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
        HrvoTestVariants.LOCAL_MINIMA_OPEN_END.name
    ]

    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=20,
    )


def test_robot_avoiding_ball_obstacle(simulated_test_runner):
    # Always Validation
    always_validation_sequence_set = [[]]

    setup_param = DRIVE_IN_STRAIGHT_LINE_PARAMS[
        HrvoTestVariants.AVOIDING_BALL_OBSTACLE.name
    ]

    eventually_validation_sequence_set = get_reached_destination_validation(setup_param)

    simulated_test_runner.run_test(
        setup=lambda world_dict: hrvo_test_setup(world_dict, simulated_test_runner),
        params=[setup_param],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=20,
    )


def get_reached_destination_validation(setup_param):
    destination = setup_param["destination"]
    threshold = 0.05
    # Eventually Validation
    return [
        [
            RobotEventuallyEntersRegionAndStops(
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
    ]


def get_move_update_control_params(
    robot_id, destination, desired_orientation, final_speed
):
    params = AssignedTacticPlayControlParams()
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
