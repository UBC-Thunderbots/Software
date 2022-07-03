import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class SpecificRobotEntersRegion(Validation):

    """Checks if a Robot enters any of the provided regions."""

    def __init__(self, robot_id, team: Team, regions=None):
        self.regions = regions if regions else []
        self.robot_id = robot_id
        self.team_color = team

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the robot enters the provided regions

        :param world: The world msg to validate
        :returns: FAILING until a robot enters any of the regions
                  PASSING when a robot enters
        """
        for region in self.regions:
            if team_color == Team.BLUE:
                robot = next(
                    (
                        robot_
                        for robot_ in world.friendly_team.team_robots
                        if robot_.id == robot_id
                    ),
                    None,
                )
            else:
                robot = next(
                    (
                        robot_
                        for robot_ in world.enemy_team.team_robots
                        if robot_.id == robot_id
                    ),
                    None,
                )

            if robot is None:
                return ValidationStatus.FAILING

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
    SpecificRobotEventuallyEntersRegion,
    SpecificRobotEventuallyExitsRegion,
    SpecificRobotAlwaysStaysInRegion,
    SpecificRobotNeverEntersRegion,
) = create_validation_types(SpecificRobotEntersRegion)
