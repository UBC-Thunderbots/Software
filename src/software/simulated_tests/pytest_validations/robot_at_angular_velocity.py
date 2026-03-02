import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.pytest_validations.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class RobotAtAngularVelocity(Validation):
    """Checks if a robot is at a specific angular velocity"""

    def __init__(self, robot_id, angular_velocity, threshold=0.1):
        """Constructs the validation object

        :param robot_id: The ID of the robot to check
        :param angular_velocity: The expected angular velocity (tbots_cpp.Angle)
        :param threshold: The tolerance in radians/s for the angular velocity check
        """
        self.robot_id = robot_id
        self.angular_velocity = angular_velocity
        self.threshold = threshold

    @override
    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the robot is at the expected angular velocity

        :param world: The world msg to validate
        :return: FAILING when the robot is not at the expected angular velocity
                 PASSING when the robot is at the expected angular velocity
        """
        for robot in world.friendly_team.team_robots:
            if robot.id == self.robot_id:
                robot_ang_vel = tbots_cpp.Angle.fromRadians(
                    robot.current_state.global_angular_velocity.radians_per_second
                )
                angle_diff = abs(
                    robot_ang_vel.toRadians() - self.angular_velocity.toRadians()
                )
                if angle_diff < self.threshold:
                    return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the angular velocity of the robot for visualization

        :param world: The world msg to create validation geometry from
        :return: ValidationGeometry showing the robot's angular velocity
        """
        # TODO (#2558): create validation geometry
        return create_validation_geometry()

    @override
    def __repr__(self):
        return f"Check that robot {self.robot_id} is at angular velocity {self.angular_velocity} (threshold={self.threshold})"


(
    RobotEventuallyAtAngularVelocity,
    RobotEventuallyNotAtAngularVelocity,
    RobotAlwaysAtAngularVelocity,
    RobotNeverAtAngularVelocity,
) = create_validation_types(RobotAtAngularVelocity)
