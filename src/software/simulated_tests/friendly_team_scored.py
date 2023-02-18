import software.python_bindings as tbots
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class FriendlyTeamScored(Validation):

    """Checks if a ball enters the enemy goal."""

    def __init__(self):
        self.region = tbots.Field.createSSLDivisionBField().enemyGoal()

    def get_validation_status(self, world) -> ValidationStatus:
        """Checks if the ball enters the provided regions

        :param world: The world msg to validate
        :returns: FAILING until a ball enters the enemy goal
                  PASSING when a ball enters
        """

        if tbots.contains(
            self.region, tbots.createPoint(world.ball.current_state.global_position)
        ):
            return ValidationStatus.PASSING

        return ValidationStatus.FAILING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create v alidation geometry from
        :returns: ValidationGeometry containing geometry to visualize

        """
        return create_validation_geometry([self.region])

    def __repr__(self):
        return "Checking ball in " + ",".join(repr(self.region))


(
    FriendlyTeamEventuallyScored,
    _FriendlyTeamGoalEventuallyRemoved,  # These two don't make much sense
    _FriendlyTeamAlwaysScored,  # These two don't make much sense
    FriendlyTeamNeverScored,
) = create_validation_types(FriendlyTeamScored)
