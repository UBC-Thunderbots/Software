import math

import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class BallMovesToFromRobot(Validation):
    """Checks if a ball is moving towards or away from a robot"""

    def __init__(
        self, robot_id, to_robot, ticks_to_wait
    ):
        self.robot_id = robot_id
        self.to_robot = to_robot
        self.
        self.distance_to_robot = None

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball stops in the provided regions

        :param world: The world msg to validate
        :returns: FAILING until a ball stops in any of the regions
                  PASSING when a ball stops in a region
        """
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                curr_ball_robot_dist = math.hypot(
                    world.ball.current_state.global_position.x_meters - robot.current_state.global_position.x_meters,
                    world.ball.current_state.global_position.y_meters - robot.current_state.global_position.y_meters
                )
                print(curr_ball_robot_dist)
                print(self.distance_to_robot)
                if self.distance_to_robot is not None:
                    if self.to_robot and curr_ball_robot_dist < self.distance_to_robot - 0.07:
                        print("HERE?")
                        return ValidationStatus.PASSING
                    elif curr_ball_robot_dist > self.distance_to_robot + 0.07:
                        print("HERE?")
                        return ValidationStatus.PASSING

                self.distance_to_robot = curr_ball_robot_dist

        print("?????")
        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry()

    def __repr__(self):
        return "Checking ball stops in regions "


(
    BallEventuallyMovesToFromRobot,
    BallEventuallyStops,
    BallAlwaysMovesToFromRobot,
    BallNeverMovesToFromRobot,
) = create_validation_types(BallMovesToFromRobot)
