import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotEntersRegion(Validation):

    """Checks if a Robot enters any of the provided regions."""

    def __init__(self, regions=None, count=1):
        self.regions = regions if regions else []
        self.count = count

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if _any_ robot enters the provided regions

        :param world: The world msg to validate
        :returns: FAILING until a robot enters any of the regions
                  PASSING when a robot enters
        """
        for region in self.regions:
            for robot in world.friendly_team.team_robots:
                if tbots.contains(
                    region, tbots.createPoint(robot.current_state.global_position)
                ):
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
    RobotEventuallyEntersRegion,
    RobotEventuallyExitsRegion,
    RobotAlwaysStaysInRegion,
    RobotNeverEntersRegion,
) = create_validation_types(RobotEntersRegion)


class NumberOfRobotsEntersRegion(Validation):

    """Checks if a certain number of Robots enters a specific region."""

    def __init__(self, region, req_robot_cnt):
        self.region = region
        self.req_robot_cnt = req_robot_cnt
        # map to keep track of robot positions
        self.robot_in_zone = {}

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if a specific number of robots enter the provided region

        :param world: The world msg to validate
        :returns: FAILING until req_robot_cnt robots enter the region
                  PASSING when req_robot_cnt robots enters
        """
        # Update the map with latest robot status
        for robot in world.friendly_team.team_robots:
            self.robot_in_zone[robot.id] = tbots.contains(
                self.region, tbots.createPoint(robot.current_state.global_position)
            )
        # Check if there are at least req_robot_cnt number of robots in zone
        curr_cnt = 0
        for robot_id in self.robot_in_zone:
            if self.robot_in_zone[robot_id]:
                curr_cnt += 1

        # Validate on curr_cnt
        if curr_cnt == self.req_robot_cnt:
            return ValidationStatus.PASSING
        else:
            return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """
        (override) shows region to enter
        """
        return create_validation_geometry([self.region])

    def __repr__(self):
        return (
            "Check for "
            + str(self.req_robot_cnt)
            + " robots in region "
            + ",".join(repr(self.region))
        )


(
    NumberOfRobotsEventuallyEntersRegion,
    NumberOfRobotsEventuallyExitsRegion,
    NumberOfRobotsAlwaysStaysInRegion,
    NumberOfRobotsNeverEntersRegion,
) = create_validation_types(NumberOfRobotsEntersRegion)
