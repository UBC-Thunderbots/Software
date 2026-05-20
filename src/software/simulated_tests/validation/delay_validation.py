from typing import override

from proto.validation_pb2 import ValidationStatus, ValidationType, ValidationGeometry
from software.py_constants import DEFAULT_SIMULATOR_TICK_RATE_SECONDS_PER_TICK
from software.simulated_tests.validation.validation import (
    Validation,
)


class DelayValidation(Validation):
    def __init__(self, delay_s, validation):
        """A validation wrapper that adds a delay to given validation before being evaluated"""
        self.delay_s = delay_s
        self.ticks_so_far = 0
        self.delay_ticks = int(delay_s / DEFAULT_SIMULATOR_TICK_RATE_SECONDS_PER_TICK)
        self.validation = validation

    @override
    def get_validation_status(self, world, simulator_state=None) -> ValidationStatus:
        """Checks validation after delay finished. If during delay, returns default validation status.

        :param world: The world msg to validate
        :return: If during delay, defaults to FAILING for eventually validation, PASSING for always validations.
                 If after delay, returns validation status of given validation.
        """
        self.ticks_so_far += 1
        if self.ticks_so_far > self.delay_ticks:
            return self.validation.get_validation_status(world)
        else:
            return (
                ValidationStatus.PASSING
                if self.validation.get_validation_type() == ValidationType.ALWAYS
                else ValidationStatus.FAILING
            )

    @override
    def get_validation_geometry(self, world) -> ValidationGeometry:
        return self.validation.get_validation_geometry(world)

    @override
    def __repr__(self):
        return f"Delayed validation after {self.delay_s} seconds for {self.validation.__repr__()}"

    @override
    def get_validation_type(self, world) -> ValidationType:
        return self.validation.get_validation_type()
