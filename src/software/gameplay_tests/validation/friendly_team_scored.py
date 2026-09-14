from typing import override

import proto.import_all_protos as protos
import software.python_bindings as tbots_cpp
from software.gameplay_tests.validation.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class FriendlyTeamScored(Validation):
    """Checks if a ball enters the enemy goal."""

    def __init__(self):
        self.region = tbots_cpp.Field.createSSLDivisionBField().enemyGoal()

    @override
    def get_validation_status(self, world) -> protos.ValidationStatus:
        """Checks if the ball enters the provided regions

        :param world: The world msg to validate
        :return: FAILING until a ball enters the enemy goal
                 PASSING when a ball enters
        """
        if tbots_cpp.contains(
            self.region, tbots_cpp.createPoint(world.ball.current_state.global_position)
        ):
            return protos.ValidationStatus.PASSING

        return protos.ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> protos.ValidationGeometry:
        """Returns the underlying geometry this validation is checking

        :param world: The world msg to create v alidation geometry from
        :return: ValidationGeometry containing geometry to visualize
        """
        return create_validation_geometry([self.region])

    @override
    def __repr__(self):
        return "Checking ball in " + repr(self.region)


(
    FriendlyTeamEventuallyScored,
    _FriendlyTeamGoalEventuallyRemoved,  # These two don't make much sense
    _FriendlyTeamAlwaysScored,  # These two don't make much sense
    FriendlyTeamNeverScored,
) = create_validation_types(FriendlyTeamScored)
