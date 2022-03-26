import software.geom.geometry as tbots_geom
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class RobotEntersRegion(Validation):

    """Checks if a Robot enters any of the provided regions."""

    def __init__(self, regions=None):
        self.regions = regions if regions else []

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if _any_ robot enters the provided regions

        :param world: The world msg to validate
        :returns: FAILING until a robot enters any of the regions
                  PASSING when a robot enters
        """
        for region in self.regions:
            for robot in world.friendly_team.team_robots:
                if tbots_geom.contains(
                    region, tbots_geom.createPoint(robot.current_state.global_position)
                ):
                    return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create validation geometry from
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry(self.regions)

    def __repr__(self):
        return "Check for robot in regions " + ",".join(
            repr(region) for region in self.regions
        )


(
    RobotEventuallyEntersRegion,
    RobotEventuallyExitsRegion,
    RobotStaysInRegion,
    RobotNeverEntersRegion,
) = create_validation_types(RobotEntersRegion)
