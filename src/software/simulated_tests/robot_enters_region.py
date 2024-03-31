import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class MinNumberOfRobotsEntersRegion(Validation):

    """Checks if a certain number of Robots enters a specific set of regions."""

    def __init__(self, regions, req_robot_cnt):
        self.regions = regions
        self.req_robot_cnt = req_robot_cnt
        # map to keep track of robot positions
        self.robot_in_zone = {}

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if a specific number of robots enter the provided set of regions

        :param world: The world msg to validate
        :returns: FAILING until req_robot_cnt robots enter the set of regions
                  PASSING when req_robot_cnt robots enter the set of regions
        # """
        robots_in_regions = set()
        for region in self.regions:
            for robot in world.friendly_team.team_robots:
                if tbots_cpp.contains(
                    region, tbots_cpp.createPoint(robot.current_state.global_position)
                ):
                    robots_in_regions.add(robot.id)

        # Validate on length of set robots_in_regions
        if len(robots_in_regions) >= self.req_robot_cnt:
            return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) shows region to enter
        """
        return create_validation_geometry(self.regions)

    def __repr__(self):
        return (
            "Check for "
            + str(self.req_robot_cnt)
            + " robots in region "
            + ",".join(repr(self.regions))
        )


(
    NumberOfRobotsEventuallyEntersRegion,
    NumberOfRobotsEventuallyExitsRegion,
    NumberOfRobotsAlwaysStaysInRegion,
    NumberOfRobotsNeverEntersRegion,
) = create_validation_types(MinNumberOfRobotsEntersRegion)


class RobotEntersRegion(MinNumberOfRobotsEntersRegion):
    def __init__(self, regions):
        super(RobotEntersRegion, self).__init__(regions, 1)


(
    RobotEventuallyEntersRegion,
    RobotEventuallyExitsRegion,
    RobotAlwaysStaysInRegion,
    RobotNeverEntersRegion,
) = create_validation_types(RobotEntersRegion)
