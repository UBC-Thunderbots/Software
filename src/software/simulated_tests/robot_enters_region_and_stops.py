import math
import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.robot_enters_region import RobotEntersRegion
from software.simulated_tests.validation import create_validation_types


class RobotEntersRegionAndStops(RobotEntersRegion):

    """Checks if a specific robot enters the provided regions and stops there for the provided time"""

    def __init__(self, robot_id, regions=None, num_ticks=1):
        """
        Constructs the base validation with the given regions
        Sets booleans indicating validation stage to default

        :param robot_id: the id of the robot being validated
        :param regions: the regions the robot has to enter
        :param num_ticks: the time the robot has to stay stationary for in the regions
        """
        super().__init__(regions=regions)
        self.robot_id = robot_id
        self.num_ticks = num_ticks
        self.ticks_so_far = 0

        self.is_stationary = True
        self.in_region = True

    def get_validation_status(self, world) -> ValidationStatus:
        """
        Checks the base validation of RobotEntersRegion
        Then checks if a specific robot id is in the provided regions
        Then checks if that robot is stationary within a threshold for the provided number of ticks

        Sets booleans about the state of the validation for logging

        :param world: the world message to validate
        :return: FAILING until the specific robot enters the provided regions
                 and is stationary there for the provided time
                 PASSING when a robot is stationary in the region
        """
        base_validation_status = super().get_validation_status(world)

        if base_validation_status == ValidationStatus.PASSING:
            self.in_region = True
            for robot in world.friendly_team.team_robots:
                if robot.id == self.robot_id:
                    if math.hypot(
                        robot.current_state.global_velocity.x_component_meters,
                        robot.current_state.global_velocity.y_component_meters,
                    ) < math.pow(10, -2):
                        self.ticks_so_far = self.ticks_so_far + 1
                        self.is_stationary = True
                        if self.ticks_so_far >= self.num_ticks:
                            return ValidationStatus.PASSING
                    else:
                        self.ticks_so_far = 0
                        self.is_stationary = False
        else:
            self.in_region = False

        return ValidationStatus.FAILING

    def __repr__(self):
        """
        Returns a string representing the stage of validation that failed
        Either the robot has not entered the region yet, or it has but is not stationary
        """
        if not self.is_stationary:
            return f"Check that robot {self.robot_id} in correct region is stationary"

        if not self.in_region:
            return f"Check for stationary robot {self.robot_id} in regions " + ",".join(
                repr(region) for region in self.regions
            )

        return super().__repr__()

(
    RobotEventuallyEntersRegionAndStops,
    RobotEventuallyExitsRegionAndMoves,
    RobotAlwaysStaysInRegionStationary,
    RobotNeverEntersRegionAndMoves,
) = create_validation_types(RobotEntersRegionAndStops)
