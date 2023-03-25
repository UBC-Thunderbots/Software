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
    def __init__(self):
        self.fouled_robots = []

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
        robot_1_pos = tbots.createVector(
            Vector(
                x_component_meters=robot1.current_state.global_position.x_meters,
                y_component_meters=robot1.current_state.global_position.y_meters,
            )
        )

        robot_2_pos = tbots.createVector(
            Vector(
                x_component_meters=robot2.current_state.global_position.x_meters,
                y_component_meters=robot2.current_state.global_position.y_meters,
            )
        )

        robot_1_vel = tbots.createVector(
            Vector(
                x_component_meters=robot1.current_state.global_velocity.x_component_meters,
                y_component_meters=robot1.current_state.global_velocity.y_component_meters,
            )
        )

        robot_2_vel = tbots.createVector(
            Vector(
                x_component_meters=robot2.current_state.global_velocity.x_component_meters,
                y_component_meters=robot2.current_state.global_velocity.y_component_meters,
            )
        )

        # check if robots are colliding
        if (robot_1_pos - robot_2_pos).length() < ROBOT_MAX_RADIUS_METERS * 2 + 0.005:
            # check if the projection of the difference between their velocities
            # onto the line between their positions > 1.5
            # if this case is true, robots are considered to have collided
            if (robot_1_vel - robot_2_vel).dot(robot_1_pos - robot_2_pos) / (
                robot_1_pos - robot_2_pos
            ).length() > 1.5:
                # checks if the absolute speed difference between the robots is < 0.3
                if robot_1_vel.length() - robot_2_vel.length() < 0.3:
                    self.fouled_robots.extend([robot1.id, robot2.id])
                else:
                    # checks which robot is faster (that one will get a foul)
                    if robot_1_vel.length() > robot_2_vel.length():
                        self.fouled_robots.extend([robot1.id])
                    else:
                        self.fouled_robots.extend([robot2.id])

                return True

        return False

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
