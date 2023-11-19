import numpy as np
from software.py_constants import *
import software.python_bindings as tbots
from proto.import_all_protos import *
from software.simulated_tests.validation import (
    Validation,
    create_validation_types,
    create_validation_geometry,
)


class RobotsDoNotCollide(Validation):
    """
    Checks if any 2 robots have collided
    """

    ROBOT_COLLISION_BUFFER_M = 0.005

    def __init__(self):
        """
        Initialised the list of robots that have fouled to empty
        2 robots can collide with only 1 of them fouling
        """
        self.fouled_robots = []

    def get_validation_status(self, world: World) -> ValidationStatus:
        """
        Checks if any 2 robots in the world have collided
        :param world: the World message to validate
        :return: FAILING if any 2 robots have collided
                 PASSING if no robots have collided
        """
        robots = list(world.friendly_team.team_robots) + list(
            world.enemy_team.team_robots
        )
        for i in range(0, len(robots) - 1):
            for j in range(i + 1, len(robots)):
                if self.check_robot_collision(robots[i], robots[j]):
                    return ValidationStatus.FAILING

        return ValidationStatus.PASSING

    def check_robot_collision(self, robot1: Robot, robot2: Robot):
        """
        Helper function to check if 2 robots have collided
        Also determines which robots have committed a foul
        :param robot1: first robot
        :param robot2: second robot
        :return: True if a collision occurs
                 False if no collision
                 Adds robots which fouled to self.fouled_robots
        """

        # robot positions
        robot1_pos = tbots.createVector(
            Vector(
                x_component_meters=robot1.current_state.global_position.x_meters,
                y_component_meters=robot1.current_state.global_position.y_meters,
            )
        )

        robot2_pos = tbots.createVector(
            Vector(
                x_component_meters=robot2.current_state.global_position.x_meters,
                y_component_meters=robot2.current_state.global_position.y_meters,
            )
        )

        # robot velocities
        robot1_vel = tbots.createVector(
            Vector(
                x_component_meters=robot1.current_state.global_velocity.x_component_meters,
                y_component_meters=robot1.current_state.global_velocity.y_component_meters,
            )
        )

        robot2_vel = tbots.createVector(
            Vector(
                x_component_meters=robot2.current_state.global_velocity.x_component_meters,
                y_component_meters=robot2.current_state.global_velocity.y_component_meters,
            )
        )

        # check if robots are colliding
        if (
            robot1_pos - robot2_pos
        ).length() < ROBOT_MAX_RADIUS_METERS * 2 + self.ROBOT_COLLISION_BUFFER_M:
            # check if the projection of the difference between their velocities
            # onto the line between their positions > 1.5
            # if this case is true, robots are considered to have collided
            # but still have to determine which ones have fouled
            if (robot1_vel - robot2_vel).dot(robot1_pos - robot2_pos) / (
                robot1_pos - robot2_pos
            ).length() > 1.5:
                self.check_fouled_robots(robot1.id, robot1_vel, robot2.id, robot2_vel)
                return True

        return False

    def check_fouled_robots(
        self, robot1_id: int, robot1_vel: Vector, robot2_id: int, robot2_vel: Vector
    ):
        """
        Determines which of the 2 robots have fouled based on their speed
        and adds them to the fouled robots list
        The logic on which robot(s) get a foul is from the official SSL rules
        https://robocup-ssl.github.io/ssl-rules/sslrules.html#_crashing
        :param robot1_id: the id of the first robot
        :param robot1_vel: the velocity of the first robot
        :param robot2_id: the id of the second robot
        :param robot2_vel: the velocity of the second robot
        """
        # checks if the absolute speed difference between the robots is < 0.3
        # if so, both get a foul
        if abs(robot1_vel.length() - robot2_vel.length()) < 0.3:
            self.fouled_robots.extend([robot1_id, robot2_id])
        else:
            # checks which robot is faster (that one will get a foul)
            if robot1_vel.length() > robot2_vel.length():
                self.fouled_robots.extend([robot1_id])
            else:
                self.fouled_robots.extend([robot2_id])

    def get_validation_geometry(self, world: World) -> ValidationGeometry:
        """
        Returns a list of circles indicating the boundary of each robot in the world
        :param world: the world message to create geometry for
        :return: ValidationGeometry with a list of circles around each robot
        """
        return create_validation_geometry(
            [
                tbots.Circle(
                    tbots.Point(
                        robot.current_state.global_position.x_meters,
                        robot.current_state.global_position.y_meters,
                    ),
                    ROBOT_MAX_RADIUS_METERS + 0.0025,
                )
                for robot in list(world.friendly_team.team_robots)
                + list(world.enemy_team.team_robots)
            ]
        )

    def __repr__(self):
        """
        String representation of the validation

        If any robots have fouled, returns a string with all their ids
        :return: string with robot ids that have fouled,
                 or default message if none have fouled
        """
        if self.fouled_robots:
            return f"Robots that fouled: {str(self.fouled_robots)}"

        return f"Check that no robots are colliding with each other"


(
    RobotsEventuallyDoesNotCollide,
    RobotsEventuallyCollides,
    RobotsAlwaysDoesNotCollide,
    RobotsAlwaysCollides,
) = create_validation_types(RobotsDoNotCollide)
