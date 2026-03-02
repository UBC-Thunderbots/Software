import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.pytest_validations.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class RobotAtOrientation(Validation):
    """Checks if a robot is at a specific orientation"""

    LINE_LENGTH = 0.3

    def __init__(self, robot_id, orientation, threshold=0.1):
        """Constructs the validation object

        :param robot_id: The ID of the robot to check
        :param orientation: The expected orientation (tbots_cpp.Angle)
        :param threshold: The tolerance in radians for the orientation check
        """
        self.robot_id = robot_id
        self.orientation = orientation
        self.threshold = threshold

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the robot is at the expected orientation

        :param world: The world msg to validate
        :return: FAILING when the robot is not at the expected orientation
                 PASSING when the robot is at the expected orientation
        """
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                robot_angle = tbots_cpp.Angle.fromRadians(
                    robot.current_state.global_orientation.radians
                )
                angle_diff = abs(robot_angle.toRadians() - self.orientation.toRadians())
                if angle_diff < self.threshold:
                    return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the orientation of the robot for visualization

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry containing the robot's orientation
        """
        # TODO (#2558): create better validation geometry
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                robot_pos = tbots_cpp.createPoint(robot.current_state.global_position)
                direction = tbots_cpp.Vector(1, 0).rotate(self.orientation)
                end_point = robot_pos + direction * self.LINE_LENGTH
                return create_validation_geometry(
                    [tbots_cpp.Segment(robot_pos, end_point)]
                )
        return create_validation_geometry()

    @override
    def __repr__(self):
        return f"Check that robot {self.robot_id} is at orientation {self.orientation} (threshold={self.threshold})"


(
    RobotEventuallyAtOrientation,
    RobotEventuallyNotAtOrientation,
    RobotAlwaysAtOrientation,
    RobotNeverAtOrientation,
) = create_validation_types(RobotAtOrientation)
