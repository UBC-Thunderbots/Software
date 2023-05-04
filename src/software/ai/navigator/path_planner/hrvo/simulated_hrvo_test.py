import pytest
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from software.simulated_tests.robot_enters_region_and_stops import *
from software.simulated_tests.avoid_collisions import *
import software.python_bindings as tbots
from proto.ssl_gc_common_pb2 import Team
from proto.message_translation.tbots_protobuf import create_world_state_from_state


def get_zig_zag_params(front_wall_x, robot_y_delta, num_walls):
    """
    Gets the test params to cause movement in a zig zag pattern
    due to multiple walls of enemy robots

    :param front_wall_x: the x position of the first wall
    :param robot_y_delta: the vertical separation between each robot in the wall
    :param num_walls: the number of walls to use
    :return: positions and destinations of friendly and enemy robots to cause
             a zig zag movement
    """
    return (
        [Point(x_meters=front_wall_x - 0.5, y_meters=0)],
        [Point(x_meters=front_wall_x + 3 + 1.5, y_meters=0)],
        [],
        [
            Point(x_meters=x_meters, y_meters=float(y_meters) / 10)
            for x_meters in range(front_wall_x, front_wall_x + num_walls + 1, 1)
            for y_meters in range(
                0,
                (1 if x_meters % 2 == 0 else -1) * 3 * int(robot_y_delta * 10),
                (1 if x_meters % 2 == 0 else -1) * int(robot_y_delta * 10),
            )
        ],
        [],
        20,
    )


def get_robot_circle_pos(radius, num_robots, start):
    """
    Gets the test params to position robots in a circle and have them move
    along each diameter

    :param radius: the radius of the circle
    :param num_robots: the number of robots in the circle
    :param start: if True, start positioning the robots from (radius, 0)
                  else, start from (-radius, 0)
                  basically outputs robot initial and destination points based on boolean
    :return: list of positions of robots that form a circle
    """
    return [
        Point(
            x_meters=(1 if start else -1)
            * radius
            * math.cos(i * 2 * math.pi / num_robots),
            y_meters=(1 if start else -1)
            * radius
            * math.sin(i * 2 * math.pi / num_robots),
        )
        for i in range(num_robots)
    ]


def hrvo_setup(
    friendly_robots_positions,
    friendly_robots_destinations,
    friendly_robots_final_orientations,
    enemy_robots_positions,
    enemy_robots_destinations,
    simulated_test_runner,
):
    """
    Setup for the hrvo tests

    :param friendly_robots_positions: starting positions of friendly robots
    :param friendly_robots_destinations: destinations of friendly robots
    :param friendly_robots_final_orientations: final orientations of friendly robots
    :param enemy_robots_positions: starting positions of enemy robots
    :param enemy_robots_destinations: destinations of enemy robots
    :param simulated_test_runner: the current test runner being used
    """
    desired_orientation = Angle(radians=0)

    ball_initial_pos = tbots.Point(1, 2)

    ball_initial_vel = tbots.Point(0, 0)

    friendly_robots = [
        RobotState(global_position=robot_position)
        for robot_position in friendly_robots_positions
    ]

    enemy_robots = [
        RobotState(global_position=robot_position)
        for robot_position in enemy_robots_positions
    ]

    # Game Controller Setup
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.STOP, team=Team.UNKNOWN
    )
    simulated_test_runner.gamecontroller.send_ci_input(
        gc_command=Command.Type.FORCE_START, team=Team.BLUE
    )

    blue_params = AssignedTacticPlayControlParams()

    for index, destination in enumerate(friendly_robots_destinations):
        blue_params = get_move_update_control_params(
            index,
            destination,
            friendly_robots_final_orientations[index]
            if friendly_robots_final_orientations
            else desired_orientation,
            0,
            params=blue_params,
        )

    simulated_test_runner.blue_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, blue_params
    )

    # Setup no tactics on the enemy side
    yellow_params = AssignedTacticPlayControlParams()

    for index, destination in enumerate(enemy_robots_destinations):
        yellow_params = get_move_update_control_params(
            index, destination, Angle(radians=0), 0, params=yellow_params
        )

    simulated_test_runner.yellow_full_system_proto_unix_io.send_proto(
        AssignedTacticPlayControlParams, yellow_params
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


@pytest.mark.parametrize(
    "friendly_robot_positions,friendly_robot_destinations,friendly_robots_final_orientations,"
    + "enemy_robots_positions,enemy_robots_destinations,timeout_s",
    [
        # robot moving straight with no obstacles
        (
            [Point(x_meters=-2.5, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [],
            [],
            [],
            10,
        ),
        # robot moving straight with no obstacles while turning from 0 to 180 degrees
        (
            [Point(x_meters=-2.5, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [Angle(radians=math.pi)],
            [],
            [],
            10,
        ),
        # robot moving straight with a moving enemy robot behind it
        (
            [Point(x_meters=-2.3, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [],
            [Point(x_meters=-2.5, y_meters=0)],
            [Point(x_meters=-2.1, y_meters=0)],
            10,
        ),
        # robot moving straight with a moving enemy robot to its side
        (
            [Point(x_meters=-2.5, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [],
            [Point(x_meters=-1, y_meters=-0.8)],
            [Point(x_meters=1, y_meters=0)],
            10,
        ),
        # robot moving straight with a moving enemy robot moving straight towards it
        (
            [Point(x_meters=-2.5, y_meters=0)],
            [Point(x_meters=2.8, y_meters=0)],
            [],
            [Point(x_meters=2.8, y_meters=0)],
            [Point(x_meters=2.5, y_meters=0)],
            10,
        ),
        # robot moving with a stationary friendly robot in front of it
        (
            [Point(x_meters=-2.5, y_meters=0), Point(x_meters=2, y_meters=0)],
            [Point(x_meters=2.7, y_meters=0), Point(x_meters=2, y_meters=0)],
            [],
            [],
            [],
            12,
        ),
        # robot moving with a stationary enemy robot in front of it
        (
            [Point(x_meters=0.7, y_meters=0)],
            [Point(x_meters=2, y_meters=0)],
            [],
            [Point(x_meters=1, y_meters=0)],
            [],
            12,
        ),
        # robot moving straight with a moving friendly robot moving straight towards it
        (
            [Point(x_meters=-2.5, y_meters=0), Point(x_meters=2.7, y_meters=0)],
            [Point(x_meters=2.7, y_meters=0), Point(x_meters=-2.5, y_meters=0)],
            [],
            [],
            [],
            10,
        ),
        # robot moving straight with a moving friendly robot moving straight towards it
        # while both are turning from 0 to 180 degrees
        (
            [Point(x_meters=-2.5, y_meters=0), Point(x_meters=2.7, y_meters=0)],
            [Point(x_meters=2.7, y_meters=0), Point(x_meters=-2.5, y_meters=0)],
            [Angle(radians=math.pi), Angle(radians=0)],
            [],
            [],
            10,
        ),
        # robot moving with a 3 enemy robot wall in front of it
        (
            [Point(x_meters=0, y_meters=0)],
            [Point(x_meters=2.7, y_meters=0)],
            [],
            [
                Point(x_meters=1, y_meters=0),
                Point(x_meters=1, y_meters=0.3),
                Point(x_meters=1, y_meters=-0.3),
            ],
            [],
            10,
        ),
        # robot moving with various stationary enemy robots in front of it
        # walls generated by range(start_x, start_x + step * num_robots, step)
        (
            [Point(x_meters=2.8, y_meters=-1)],
            [Point(x_meters=2.8, y_meters=1)],
            [],
            [
                Point(x_meters=float(x_meters) / 10, y_meters=0)
                for x_meters in range(20, 32, 2)
            ],
            [],
            25,
        ),
        # robot moving in a local minima (enemy robots in a curve around it)
        (
            [Point(x_meters=0.7, y_meters=0)],
            [Point(x_meters=2.7, y_meters=1)],
            [],
            [
                Point(x_meters=1, y_meters=0),
                Point(x_meters=1, y_meters=0.3),
                Point(x_meters=1, y_meters=0.6),
                Point(x_meters=0.7, y_meters=0.6),
                Point(x_meters=1, y_meters=-0.3),
                Point(x_meters=1, y_meters=-0.6),
                Point(x_meters=0.7, y_meters=-0.6),
            ],
            [],
            25,
        ),
        # robot moving in a local minima (enemy robots in a curve around it) with an opening in the middle
        (
            [Point(x_meters=0.7, y_meters=0)],
            [Point(x_meters=2.7, y_meters=1)],
            [],
            [
                Point(x_meters=1.5, y_meters=0),
                Point(x_meters=1, y_meters=0.3),
                Point(x_meters=1, y_meters=0.6),
                Point(x_meters=0.7, y_meters=0.6),
                Point(x_meters=1, y_meters=-0.3),
                Point(x_meters=1, y_meters=-0.6),
                Point(x_meters=0.7, y_meters=-0.6),
            ],
            [],
            10,
        ),
        # robot moving in a zig zag path around enemy robots
        get_zig_zag_params(-2, 0.3, 3),
        # friendly robots in a circle moving along each diameter
        (
            get_robot_circle_pos(1.5, 8, True),
            get_robot_circle_pos(1.5, 8, False),
            [],
            [],
            [],
            15,
        ),
        # friendly robots in a circle moving along each diameter
        # while turning from 0 to 180 degrees
        (
            get_robot_circle_pos(1.5, 8, True),
            get_robot_circle_pos(1.5, 8, False),
            [Angle(radians=math.pi) for i in range(8)],
            [],
            [],
            20,
        ),
        # half enemy half friendly robots in a circle moving along each diameter
        (
            [
                pos
                for index, pos in enumerate(get_robot_circle_pos(1.5, 8, True))
                if index % 2 == 0
            ],
            [
                pos
                for index, pos in enumerate(get_robot_circle_pos(1.5, 8, False))
                if index % 2 == 0
            ],
            [],
            # both the start and end positions are the same values
            # but for some reason it works when actually running the test
            # so just gonna leave this here
            [
                pos
                for index, pos in enumerate(get_robot_circle_pos(1.5, 8, True))
                if index % 2 == 1
            ],
            [
                pos
                for index, pos in enumerate(get_robot_circle_pos(1.5, 8, True))
                if index % 2 == 1
            ],
            10,
        ),
    ],
    ids=[
        "moving_straight_no_obstacles",
        "moving_straight_and_turning",
        "moving_straight_with_enemy_behind",
        "moving_straight_with_enemy_to_side",
        "moving_straight_enemy_collision",
        "moving_straight_with_stationary_friendly_in_front",
        "moving_straight_with_stationary_enemy_in_front",
        "moving_straight_friendly_collision",
        "moving_straight_friendly_collision_and_turning",
        "moving_with_3_enemy_robot_wall",
        "moving_with_horizontal_enemy_wall",
        "moving_in_local_minima",
        "moving_in_local_minima_with_opening",
        "moving_in_zig_zag",
        "friendly_robots_in_circle",
        "friendly_robots_in_circle_with_turning",
        "mixed_robots_in_circle",
    ],
)
def test_robot_movement(
    simulated_test_runner,
    friendly_robot_positions,
    friendly_robot_destinations,
    friendly_robots_final_orientations,
    enemy_robots_positions,
    enemy_robots_destinations,
    timeout_s,
):
    # Always Validation
    always_validation_sequence_set = [[RobotsDoNotCollide()]]

    # Eventually Validation
    eventually_validation_sequence_set = get_reached_destination_validation(
        friendly_robot_destinations
    )

    simulated_test_runner.run_test(
        setup=lambda param: hrvo_setup(
            friendly_robot_positions,
            friendly_robot_destinations,
            friendly_robots_final_orientations,
            enemy_robots_positions,
            enemy_robots_destinations,
            simulated_test_runner,
        ),
        params=[0],
        inv_eventually_validation_sequence_set=eventually_validation_sequence_set,
        inv_always_validation_sequence_set=always_validation_sequence_set,
        ag_eventually_validation_sequence_set=[[]],
        ag_always_validation_sequence_set=[[]],
        test_timeout_s=timeout_s,
    )


def get_reached_destination_validation(robot_destinations):
    """
    Constructs validation that each robot from the list of robot destinations
    reaches its destination within a certain threshold
    and stays stationary there for 15 ticks

    :param robot_destinations: the destination of each robot we want to make validations for
    :return: the validation sequence set
    """
    threshold = 0.05
    # Eventually Validation
    return [
        [
            RobotEventuallyEntersRegionAndStops(
                robot_id,
                region=tbots.Circle(
                    tbots.Point(destination.x_meters, destination.y_meters,), threshold
                ),
                num_ticks=10,
            )
        ]
        for robot_id, destination in enumerate(robot_destinations)
    ]


def get_move_update_control_params(
    robot_id, destination, desired_orientation, final_speed, params=None
):
    """
    Constructs the control params for a Move Tactic for a single robot
    with the given data
    And adds it to an existing or new AssignedTacticPlayControlParams message

    :param robot_id: the id of the robot who will be assigned these params
    :param destination: the destination of the robot
    :param desired_orientation: the desired orientation of the robot
    :param final_speed: the final speed of the robot
    :param params: AssignedTacticPlayControlParams message
                   if not None, add this robot's params to this
                   else, create a new message and add
    :return: an AssignedTacticPlayControlParams message with this robot's params added
    """

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


if __name__ == "__main__":
    pytest_main(__file__)
