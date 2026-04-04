from typing import override

from proto.validation_pb2 import ValidationStatus, ValidationType

from software.simulated_tests.validation.validation import (
    Validation,
)


class DurationValidation(Validation):
    def __init__(self, duration_s, validation):
        """A validation wrapper that adds a duration to given validation to be evaluated"""
        if validation.get_validation_type() != ValidationType.EVENTUALLY:
            raise TypeError(
                "Type of validation needs to be EVENTUALLY for DurationValidation"
            )

        tick_duration_s = 0.0166

        self.duration_s = duration_s
        self.passing_ticks = 0
        self.duration_ticks = int(duration_s / tick_duration_s)
        self.validation = validation

    @override
    def get_validation_status(self, world):
        if self.validation.get_validation_status(world) == ValidationStatus.PASSING:
            self.passing_ticks += 1
        if self.passing_ticks > self.duration_ticks:
            return ValidationStatus.PASSING
        else:
            return ValidationStatus.FAILING

    @override
    def get_validation_geometry(self, world):
        return self.validation.get_validation_geometry(world)

    @override
    def __repr__(self):
        return f"Duration validation for {self.duration_s} seconds for {self.validation.__repr__()}"

    @override
    def get_validation_type(self, world):
        return ValidationType.EVENTUALLY
