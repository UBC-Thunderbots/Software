import math
from software.py_constants import *
import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    create_validation_types,
    create_validation_geometry,
)
from software.simulated_tests.robot_enters_region import RobotEntersRegion


class RobotEntersRegionAndStops(RobotEntersRegion):
    """Checks if a robot enters the provided regions and stops there for the provided time"""

    ROBOT_MAX_STATIONARY_SPEED_M_PER_S = 0.01

    def __init__(self, regions=None, num_ticks=1):
        """
        Constructs the base validation with the given regions
        Sets boolean indicating validation stage to default

        :param regions: the regions the robot has to enter
        :param num_ticks: the time the robot has to stay stationary for in the regions
        """
        super().__init__(regions)
        self.num_ticks = num_ticks
        self.ticks_so_far = 0

        self.is_stationary = True
        self.passing_robot_id = None

    def get_validation_status(self, world) -> ValidationStatus:
        """
        Checks if a robot is in the provided region
        Then checks if that robot is stationary within a threshold for the provided number of ticks

        Sets booleans about the state of the validation for logging

        ASSUMES 1 robot per region. If there's more than 1 robot in the region the given regions can
        probably be made more specific

        :param world: the world message to validate
        :return: FAILING until the specific robot enters the provided region
                 and is stationary there for the provided time
                 PASSING when a robot is stationary in the region
        """
        robot_in_region_validation = super().get_validation_status(world)

        # if a robot is currently in the region
        if (
            robot_in_region_validation == ValidationStatus.PASSING
            and self.passing_robot is not None
        ):
            # check if robot speed is less than stationary threshold
            if (
                math.hypot(
                    self.passing_robot.current_state.global_velocity.x_component_meters,
                    self.passing_robot.current_state.global_velocity.y_component_meters,
                )
                < self.ROBOT_MAX_STATIONARY_SPEED_M_PER_S
            ):
                self.ticks_so_far = self.ticks_so_far + 1
                self.is_stationary = True
                # if the robot has been stationary for the specified num ticks
                if self.ticks_so_far >= self.num_ticks:
                    return ValidationStatus.PASSING
            else:
                # reset the num ticks since robot is moving too fast
                self.ticks_so_far = 0
                self.is_stationary = False
                return ValidationStatus.FAILING
        else:
            # reset the num ticks since robot is no longer in region
            self.ticks_so_far = 0
            self.is_stationary = False
            return robot_in_region_validation

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) shows region to enter
        """
        return create_validation_geometry(self.regions)

    def __repr__(self):
        """
        Returns a string representing the stage of validation that failed
        Either the robot has not entered the region yet, or it has but is not stationary
        """
        if not self.is_stationary:
            return f"Check that robot in correct region is stationary"

        return f"Check for stationary robot in region " + ",".join(
            repr(region) for region in self.regions
        )


(
    RobotEventuallyEntersRegionAndStops,
    RobotEventuallyExitsRegionAndMoves,
    RobotAlwaysStaysInRegionStationary,
    RobotNeverEntersRegionAndMoves,
) = create_validation_types(RobotEntersRegionAndStops)
