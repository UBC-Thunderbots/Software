import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotStopsInRegion(Validation):

    """Checks if a Robot enters any of the provided regions."""

    def __init__(self, regions=None, num_robots=1):
        self.regions = regions if regions else []
        self.num_robots = num_robots

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if _any_ robot enters the provided regions

        :param world: The world msg to validate
        :returns: FAILING until a robot enters any of the regions
                  PASSING when a robot enters
        """
        num_robots = self.num_robots
        for region in self.regions:
            for robot in world.friendly_team.team_robots:
                if tbots.contains(
                    region, tbots.createPoint(robot.current_state.global_position)
                ) and (tbots.createVector(robot.velocity).length() <= 0.01):
                    num_robots -= 1
                    if num_robots == 0:
                        return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) shows regions to enter
        """
        return create_validation_geometry(self.regions)

    def __repr__(self):
        return "Check for robot in regions " + ",".join(
            repr(region) for region in self.regions
        )


(
    RobotEventuallyStopsInRegion,
    RobotEventuallyMovesInRegion,
    RobotAlwaysStopsInRegion,
    RobotNeverStopsInRegion,
) = create_validation_types(RobotStopsInRegion)
