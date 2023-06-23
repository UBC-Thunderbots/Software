import math
from software.py_constants import *
import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    create_validation_types,
    create_validation_geometry,
)


class RobotEntersRegionAndStops:
    """Checks if a specific robot enters the provided regions and stops there for the provided time"""

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
    RobotEventuallyEntersRegionAndStops,
    RobotEventuallyExitsRegionAndMoves,
    RobotAlwaysStaysInRegionStationary,
    RobotNeverEntersRegionAndMoves,
) = create_validation_types(RobotEntersRegionAndStops)
