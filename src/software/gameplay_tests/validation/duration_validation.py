from typing import override

import proto.import_all_protos as protos
from software.gameplay_tests.validation.validation import (
    Validation,
)
from software.py_constants import DEFAULT_SIMULATOR_TICK_RATE_SECONDS_PER_TICK


class DurationValidation(Validation):
    def __init__(self, duration_s, validation):
        """A validation wrapper that adds a duration to given validation to be evaluated"""
        if validation.get_validation_type() != protos.ValidationType.EVENTUALLY:
            raise TypeError(
                "Type of validation needs to be EVENTUALLY for DurationValidation"
            )

        self.duration_s = duration_s
        self.passing_ticks = 0
        self.duration_ticks = int(
            duration_s / DEFAULT_SIMULATOR_TICK_RATE_SECONDS_PER_TICK
        )
        self.validation = validation

    @override
    def get_validation_status(self, world) -> protos.ValidationStatus:
        """Checks if validation has been consecutively PASSING for a duration.

        :param world: The world msg to validate
        :return: FAILING if given validation has not yet passed for given duration.
                 PASSING if given validation has passed for given duration.
        """
        if (
            self.validation.get_validation_status(world)
            == protos.ValidationStatus.PASSING
        ):
            self.passing_ticks += 1
        else:
            self.passing_ticks = 0

        if self.passing_ticks > self.duration_ticks:
            return protos.ValidationStatus.PASSING
        else:
            return protos.ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world) -> protos.ValidationGeometry:
        return self.validation.get_validation_geometry(world)

    @override
    def __repr__(self):
        return f"Duration validation for {self.duration_s} seconds for {self.validation.__repr__()}"

    @override
    def get_validation_type(self, world) -> protos.ValidationType:
        return protos.ValidationType.EVENTUALLY
