import math

import numpy as np
from software.py_constants import *
import software.python_bindings as tbots
from proto.import_all_protos import *
from software.simulated_tests.validation import (
    Validation,
    create_validation_types,
    create_validation_geometry,
)


class RobotDoesNotCollide(Validation):
    def get_validation_status(self, world) -> ValidationStatus:
        robots = list(world.friendly_team.team_robots) + list(
            world.enemy_team.team_robots
        )
        for i in range(0, len(robots) - 1):
            for j in range(i + 1, len(robots)):
                if self.check_robot_collision(robots[i], robots[j]):
                    return ValidationStatus.FAILING

        return ValidationStatus.PASSING

    def check_robot_collision(self, robot1, robot2):
        robot_1_pos = [
            robot1.current_state.global_position.x_meters,
            robot1.current_state.global_position.y_meters,
        ]

        robot_2_pos = [
            robot2.current_state.global_position.x_meters,
            robot2.current_state.global_position.y_meters,
        ]

        robot_1_vel = [
            robot1.current_state.global_velocity.x_component_meters,
            robot1.current_state.global_velocity.y_component_meters,
        ]

        robot_2_vel = [
            robot2.current_state.global_velocity.x_component_meters,
            robot2.current_state.global_velocity.y_component_meters,
        ]
        return (
            math.dist(robot_1_pos, robot_2_pos) < ROBOT_MAX_RADIUS_METERS * 2 + 0.005
        ) and (
            (
                math.hypot(robot_1_vel[0], robot_1_vel[1])
                - math.hypot(robot_2_vel[0], robot_2_vel[1])
                < 0.3
            )
            or (
                np.dot(
                    [robot_1_vel[0] - robot_2_vel[0], robot_1_vel[1] - robot_2_vel[1]],
                    [robot_1_pos[0] - robot_2_pos[0], robot_1_pos[1] - robot_2_pos[1]],
                )
                / math.dist(robot_1_pos, robot_2_pos)
                > 1.5
            )
        )

    def get_validation_geometry(self, world) -> ValidationGeometry:
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
        return f"Check that no robots are colliding with each other"


(
    RobotEventuallyDoesNotCollide,
    RobotEventuallyCollides,
    RobotAlwaysDoesNotCollide,
    RobotAlwaysCollides,
) = create_validation_types(RobotDoesNotCollide)
