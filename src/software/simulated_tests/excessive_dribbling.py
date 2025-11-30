import software.python_bindings as tbots_cpp
from proto.import_all_protos import *

from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)
from typing import override


class ExcessivelyDribbling(Validation):
    """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m."""

    def __init__(self):
        self.continous_dribbling_start_point = None

    @override
    def get_validation_status(
        self, world, max_dribble_length: float = 1.00
    ) -> ValidationStatus:
        """Checks if any friendly robot is excessively dribbling the ball, i.e. for over 1m.

        :param world: The world msg to validate
        :param max_dribble_length: the maximum dribble distance allowed (competition 1m)
        :return: FAILING when the robot is excessively dribbling
                 PASSING when the robot is not excessively dribbling
        """
        # Use world calculation of dribbling distance, which uses implementation
        # of initial position of BOT to final position of BALL
        if world.HasField("dribble_displacement"):
            dribble_disp = world.dribble_displacement
            dist = tbots_cpp.createSegment(dribble_disp).length()
            if dist > max_dribble_length:
                return ValidationStatus.FAILING

        return ValidationStatus.PASSING

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        """(override) Shows the max allowed dribbling circle"""
        return create_validation_geometry(
            [tbots_cpp.Circle(self.continous_dribbling_start_point, 1.0)]
            if self.continous_dribbling_start_point is not None
            else []
        )

    @override
    def __repr__(self):
        return "Check that the dribbling robot has not dribbled for more than 1m"


(
    EventuallyStopExcessivelyDribbling,
    EventuallyStartsExcessivelyDribbling,
    NeverExcessivelyDribbles,
    AlwaysExcessivelyDribbles,
) = create_validation_types(ExcessivelyDribbling)
