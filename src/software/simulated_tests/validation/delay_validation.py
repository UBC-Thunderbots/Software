from typing import override

from proto.validation_pb2 import ValidationStatus, ValidationType

from software.simulated_tests.validation.validation import (
    Validation,
)
from software.thunderscope.constants import SimulatedTestConstants


class DelayValidation(Validation):
    def __init__(self, delay_s, validation):
        """A validation wrapper that adds a delay to given validation before being evaluated"""
        self.delay_s = delay_s
        self.ticks_so_far = 0
        self.delay_ticks = int(delay_s / SimulatedTestConstants.TICK_DURATION_S)
        self.validation = validation

    @override
    def get_validation_status(self, world):
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
    def get_validation_geometry(self, world):
        return self.validation.get_validation_geometry(world)

    @override
    def __repr__(self):
        return f"Delayed validation after {self.delay_s} seconds for {self.validation.__repr__()}"

    @override
    def get_validation_type(self, world):
        return self.validation.get_validation_type()
