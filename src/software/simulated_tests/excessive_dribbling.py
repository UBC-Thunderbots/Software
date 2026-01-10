import software.python_bindings as tbots_cpp
from proto.import_all_protos import ValidationStatus, ValidationGeometry
from software.thunderscope.constants import DribblingConstants

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class ExcessivelyDribbling(Validation):
    """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m."""

    def get_validation_status(
        self,
        world,
    ) -> ValidationStatus:
        """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m.

        :param world: The world msg to validate
               estimate of max dribble distance (effective dribble distance is length - error margin)
        :return: FAILING when the robot is excessively dribbling
                 PASSING when the robot is not excessively dribbling
        """
        # Use world calculation of dribbling distance, which uses implementation
        # of initial position of BOT to final position of BALL

        if world.HasField("dribble_displacement"):
            dribble_disp = world.dribble_displacement
            dist = tbots_cpp.createSegment(dribble_disp).length()
            if dist > (
                DribblingConstants.MAX_DRIBBLING_DISTANCE
                - DribblingConstants.DRIBBLING_ERROR_MARGIN
            ):
                return ValidationStatus.FAILING

        return ValidationStatus.PASSING

    def get_validation_geometry(self, world) -> ValidationGeometry:
        """(override) Shows the max allowed dribbling circle"""
        if world.HasField("dribble_displacement"):
            dribbling_start_point = tbots_cpp.createSegment(
                world.dribble_displacement
            ).getStart()
            return create_validation_geometry(
                [tbots_cpp.Circle(dribbling_start_point, 1.0)]
            )
        return create_validation_geometry([])

    def __repr__(self):
        return "Check that the dribbling robot has not dribbled for more than 1m"


(
    EventuallyStopExcessivelyDribbling,
    EventuallyStartsExcessivelyDribbling,
    NeverExcessivelyDribbles,
    AlwaysExcessivelyDribbles,
) = create_validation_types(ExcessivelyDribbling)
