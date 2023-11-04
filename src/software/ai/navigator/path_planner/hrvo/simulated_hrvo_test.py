import pytest
from software.simulated_tests.simulated_test_fixture import (
    simulated_test_runner,
    pytest_main,
)
from software.simulated_tests.avoid_collisions import *
import software.python_bindings as tbots
from proto.ssl_gc_common_pb2 import Team
from software.py_constants import *
from proto.message_translation.tbots_protobuf import create_world_state
import math
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    create_validation_types,
    create_validation_geometry,
)


class HRVORobotEntersRegionAndStops:
    """Checks if a specific robot enters the provided regions and stops there for the provided time
    Only used for HRVO tests
    """

    ROBOT_MAX_STATIONARY_SPEED_M_PER_S = 0.01

    def __init__(self, robot_id, region=None, num_ticks=1):
        """
        Constructs the base validation with the given region
        Sets boolean indicating validation stage to default

        :param robot_id: the id of the robot being validated
        :param region: the region the robot has to enter
        :param num_ticks: the time the robot has to stay stationary for in the regions
        """
        self.region = region
        self.robot_id = robot_id
        self.num_ticks = num_ticks
        self.ticks_so_far = 0

        self.is_stationary = True

    def get_validation_status(self, world) -> ValidationStatus:
        """
        Checks if a specific robot id is in the provided region
        Then checks if that robot is stationary within a threshold for the provided number of ticks

        Sets booleans about the state of the validation for logging

        :param world: the world message to validate
        :return: FAILING until the specific robot enters the provided region
                 and is stationary there for the provided time
                 PASSING when a robot is stationary in the region
        """
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                if tbots.contains(
                    self.region, tbots.createPoint(robot.current_state.global_position)
                ):
                    if (
                        math.hypot(
                            robot.current_state.global_velocity.x_component_meters,
                            robot.current_state.global_velocity.y_component_meters,
                        )
                        < self.ROBOT_MAX_STATIONARY_SPEED_M_PER_S
                    ):
                        self.ticks_so_far = self.ticks_so_far + 1
                        self.is_stationary = True
                        if self.ticks_so_far >= self.num_ticks:
                            return ValidationStatus.PASSING
                    else:
                        self.ticks_so_far = 0
                        self.is_stationary = False

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) shows region to enter
        """
        return create_validation_geometry([self.region])

    def __repr__(self):
        """
        Returns a string representing the stage of validation that failed
        Either the robot has not entered the region yet, or it has but is not stationary
        """
        if not self.is_stationary:
            return f"Check that robot {self.robot_id} in correct region is stationary"

        return (
            f"Check for stationary robot {self.robot_id} in region "
            + ","
            + repr(self.region)
        )


(
    HRVORobotEventuallyEntersRegionAndStops,
    HRVORobotEventuallyExitsRegionAndMoves,
    HRVORobotAlwaysStaysInRegionStationary,
    HRVORobotNeverEntersRegionAndMoves,
) = create_validation_types(HRVORobotEntersRegionAndStops)


def get_zig_zag_params(front_wall_x, robot_y_delta, num_walls, wall_height):
    """
    Gets the test params to cause movement in a zig zag pattern
    due to multiple walls of enemy robots

    :param front_wall_x: the x position of the first wall
    :param robot_y_delta: the vertical separation between each robot in the wall
    :param num_walls: the number of walls to use
    :param wall_height: the height of each wall
    :return: positions and destinations of friendly and enemy robots to cause
             a zig zag movement
    """
    return (
        [tbots.Point(front_wall_x - 0.5, 0)],
        [tbots.Point(front_wall_x + 3 + 1.5, 0)],
        [],
        [
            tbots.Point(x_meters, float(y_meters) / 10)
            for x_meters in range(front_wall_x, front_wall_x + num_walls + 1, 1)
            for y_meters in range(
                0,
                (1 if x_meters % 2 == 0 else -1)
                * wall_height
                * int(robot_y_delta * 10),
                (1 if x_meters % 2 == 0 else -1) * int(robot_y_delta * 10),
            )
        ],
        [],
        20,
        False,
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
        tbots.Point(
            (1 if start else -1) * radius * math.cos(i * 2 * math.pi / num_robots),
            (1 if start else -1) * radius * math.sin(i * 2 * math.pi / num_robots),
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
    desired_orientation = tbots.Angle.fromRadians(0)

    ball_initial_pos = tbots.Point(1, 2)

    ball_initial_vel = tbots.Point(0, 0)

    # Game Controller Setup
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.STOP, is_friendly=True
    )
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.STOP, is_friendly=False
    )
    simulated_test_runner.send_gamecontroller_command(
        gc_command=Command.Type.FORCE_START, is_friendly=True
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

    simulated_test_runner.set_tactics(blue_params, True)

    # Setup no tactics on the enemy side
    yellow_params = AssignedTacticPlayControlParams()

    for index, destination in enumerate(enemy_robots_destinations):
        yellow_params = get_move_update_control_params(
            index, destination, tbots.Angle.fromRadians(0), 0, params=yellow_params
        )

    simulated_test_runner.set_tactics(yellow_params, False)

    # Setup Robots
    simulated_test_runner.simulator_proto_unix_io.send_proto(
        WorldState,
        create_world_state(
            yellow_robot_locations=enemy_robots_positions,
            blue_robot_locations=friendly_robots_positions,
            ball_location=ball_initial_pos,
            ball_velocity=ball_initial_vel,
        ),
    )


@pytest.mark.parametrize(
    "friendly_robot_positions,friendly_robot_destinations,friendly_robots_final_orientations,"
    + "enemy_robots_positions,enemy_robots_destinations,timeout_s,run_till_end",
    [
        # robot moving straight with no obstacles
        ([tbots.Point(-2.5, 0)], [tbots.Point(2.8, 0)], [], [], [], 10, False,),
        # robot moving straight with no obstacles while turning from 0 to 180 degrees
        (
            [tbots.Point(-2.5, 0)],
            [tbots.Point(2.8, 0)],
            [tbots.Angle.fromRadians(math.pi)],
            [],
            [],
            10,
            False,
        ),
        # robot moving straight with a moving enemy robot behind it
        (
            [tbots.Point(-2.3, 0)],
            [tbots.Point(2.8, 0)],
            [],
            [tbots.Point(-2.5, 0)],
            [tbots.Point(-2.1, 0)],
            10,
            False,
        ),
        # robot moving straight with a moving enemy robot to its side
        (
            [tbots.Point(-2.5, 0)],
            [tbots.Point(2.8, 0)],
            [],
            [tbots.Point(-1, -0.8)],
            [tbots.Point(1, 0)],
            10,
            False,
        ),
        # robot moving straight with a moving enemy robot moving straight towards it
        (
            [tbots.Point(-2.5, 0)],
            [tbots.Point(2.8, 0)],
            [],
            [tbots.Point(2.8, 0)],
            [tbots.Point(2.5, 0)],
            10,
            False,
        ),
        # robot moving with a stationary friendly robot in front of it
        (
            [tbots.Point(-2.5, 0), tbots.Point(2, 0)],
            [tbots.Point(2.7, 0), tbots.Point(2, 0)],
            [],
            [],
            [],
            12,
            False,
        ),
        # robot moving with a stationary enemy robot in front of it
        (
            [tbots.Point(0.7, 0)],
            [tbots.Point(2, 0)],
            [],
            [tbots.Point(1, 0)],
            [],
            12,
            False,
        ),
        # robot moving straight with a moving friendly robot moving straight towards it
        (
            [tbots.Point(-2.5, 0), tbots.Point(2.7, 0)],
            [tbots.Point(2.7, 0), tbots.Point(-2.5, 0)],
            [],
            [],
            [],
            10,
            False,
        ),
        # robot moving straight with a moving friendly robot moving straight towards it
        # while both are turning from 0 to 180 degrees
        (
            [tbots.Point(-2.5, 0), tbots.Point(2.7, 0)],
            [tbots.Point(2.7, 0), tbots.Point(-2.5, 0)],
            [tbots.Angle.fromRadians(math.pi), tbots.Angle.fromRadians(0)],
            [],
            [],
            10,
            False,
        ),
        # robot moving with a 3 enemy robot wall in front of it
        (
            [tbots.Point(0, 0)],
            [tbots.Point(2.7, 0)],
            [],
            [tbots.Point(1, 0), tbots.Point(1, 0.3), tbots.Point(1, -0.3),],
            [],
            10,
            False,
        ),
        # robot moving with various stationary enemy robots in front of it
        # walls generated by range(start_x, start_x + step * num_robots, step)
        (
            [tbots.Point(2.8, -1)],
            [tbots.Point(2.8, 1)],
            [],
            [tbots.Point(float(x_meters) / 10, 0) for x_meters in range(20, 32, 2)],
            [],
            25,
            True,
        ),
        # robot moving in a local minima (enemy robots in a curve around it)
        (
            [tbots.Point(0.7, 0)],
            [tbots.Point(2.7, 1)],
            [],
            [
                tbots.Point(1, 0),
                tbots.Point(1, 0.3),
                tbots.Point(1, 0.6),
                tbots.Point(0.7, 0.6),
                tbots.Point(1, -0.3),
                tbots.Point(1, -0.6),
                tbots.Point(0.7, -0.6),
            ],
            [],
            25,
            False,
        ),
        # robot moving in a local minima (enemy robots in a curve around it) with an opening in the middle
        (
            [tbots.Point(0.7, 0)],
            [tbots.Point(2.7, 1)],
            [],
            [
                tbots.Point(1.5, 0),
                tbots.Point(1, 0.3),
                tbots.Point(1, 0.6),
                tbots.Point(0.7, 0.6),
                tbots.Point(1, -0.3),
                tbots.Point(1, -0.6),
                tbots.Point(0.7, -0.6),
            ],
            [],
            10,
            False,
        ),
        # robot moving in a zig zag path around enemy robots
        get_zig_zag_params(-2, 0.3, 3, 5),
        # friendly robots in a circle moving along each diameter
        (
            get_robot_circle_pos(1.5, 8, True),
            get_robot_circle_pos(1.5, 8, False),
            [],
            [],
            [],
            15,
            False,
        ),
        # friendly robots in a circle moving along each diameter
        # while turning from 0 to 180 degrees
        (
            get_robot_circle_pos(1.5, 8, True),
            get_robot_circle_pos(1.5, 8, False),
            [tbots.Angle.fromRadians(math.pi) for i in range(8)],
            [],
            [],
            20,
            False,
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
            # both the start and end positions are the same values but they work
            # this is because the start positions for yellow are in the Blue Fullsystem perspective
            # but the enemy destinations are used to create a yellow Move Tactic,
            # which means the destinations given here are in yellow's perspective
            # we see blue's perspective on screen, which flips the x and y from the yellow perspective
            # so when displayed, the destination positions look correct since they have been flipped for blue
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
            False,
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
    run_till_end,
):
    # Always Validation
    always_validation_sequence_set = [[RobotsDoNotCollide()]]

    # Eventually Validation
    eventually_validation_sequence_set = (
        [[]]
        if run_till_end
        else get_reached_destination_validation(friendly_robot_destinations)
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
        run_till_end=run_till_end,
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
            HRVORobotEventuallyEntersRegionAndStops(
                robot_id,
                region=tbots.Circle(
                    tbots.Point(destination.x(), destination.y()), threshold
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
            destination=Point(x_meters=destination.x(), y_meters=destination.y()),
            final_orientation=Angle(radians=desired_orientation.toRadians()),
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
