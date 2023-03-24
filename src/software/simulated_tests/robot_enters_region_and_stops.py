import math
import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.robot_enters_region import RobotEntersRegion
from software.simulated_tests.validation import create_validation_types


class RobotEntersRegionAndStops(RobotEntersRegion):
    def __init__(self, robot_id, regions=None, num_ticks=1):
        super().__init__(regions=regions)
        self.robot_id = robot_id
        self.num_ticks = num_ticks
        self.ticks_so_far = 0

    def get_validation_status(self, world) -> ValidationStatus:
        base_validation_status = super().get_validation_status(world)

        if base_validation_status == ValidationStatus.PASSING:
            for robot in world.friendly_team.team_robots:
                if robot.id == self.robot_id:
                    if math.hypot(
                        robot.current_state.global_velocity.x_component_meters,
                        robot.current_state.global_velocity.y_component_meters,
                    ) < math.pow(10, -2):
                        self.ticks_so_far = self.ticks_so_far + 1
                        if self.ticks_so_far >= self.num_ticks:
                            return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def __repr__(self):
        return f"Check for stationary robot {self.robot_id} in regions " + ",".join(
            repr(region) for region in self.regions
        )


(
    RobotEventuallyEntersRegionAndStops,
    RobotEventuallyExitsRegionAndMoves,
    RobotAlwaysStaysInRegionStationary,
    RobotNeverEntersRegionAndMoves,
) = create_validation_types(RobotEntersRegionAndStops)
