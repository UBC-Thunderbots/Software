from typing import override

import proto.import_all_protos as protos
import software.python_bindings as tbots_cpp
from software.gameplay_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotAtPosition(Validation):
    """Checks if a specific robot is at a given position within a threshold distance."""

    def __init__(
        self, robot_id: int, position: tbots_cpp.Point, threshold: float = 0.05
    ):
        """Initializes the validation class with robot id, position, and threshold

        :param robot_id: the id of the robot to check
        :param position: the target position the robot should be at
        :param threshold: the distance threshold for the robot to be considered at position
        """
        self.robot_id = robot_id
        self.position = position
        self.threshold = threshold

    @override
    def get_validation_status(self, world) -> protos.ValidationStatus:
        """Checks if the robot is at the target position

        :param world: The world msg to validate
        :returns: PASSING if the robot is at the target position within threshold
                  FAILING otherwise
        """
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                robot_pos = tbots_cpp.createPoint(robot.current_state.global_position)
                distance = (robot_pos - self.position).length()
                if distance <= self.threshold:
                    return protos.ValidationStatus.PASSING
                break

        return protos.ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> protos.ValidationGeometry:
        """(override) shows the target position"""
        circle = tbots_cpp.Circle(self.position, self.threshold)
        return create_validation_geometry([circle])

    @override
    def __repr__(self):
        return f"Robot {self.robot_id} at position {self.position} with threshold {self.threshold}"


(
    RobotEventuallyAtPosition,
    RobotEventuallyNotAtPosition,
    RobotAlwaysAtPosition,
    RobotAlwaysNotAtPosition,
) = create_validation_types(RobotAtPosition)
