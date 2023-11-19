import pytest

import software.python_bindings as tbots
from proto.validation_pb2 import *
from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class OrValidation():
    def __init__ (self, validation):
        """An or extension to the validation function"""
        self.validation = validation

    def get_validation_status(self, world):
        for validation in self.validation: #
            if validation.get_validation_status(world) == ValidationStatus.PASSING: # should I be using ValidationStatus.PASSING or just == FAILING
                return ValidationStatus.PASSING
        return ValidationStatus.PASSING

    def get_validation_geometry(self, world):

        validation_geometry = ValidationGeometry()

        for validation in self.validation:
            validation.get_validation_geometry(world) # why is this needed
            validation_geometry.polygons += validation.polygons
            validation_geometry.circles += validation.circles
            validation_geometry.vectors += validation.vectors
            validation_geometry.segments += validation.segments

        return validation_geometry


    def get_validation_type(self, world):
        validation_type_initial = self.validation[0].get_validation_type

        for validation in self.validation:
            validation_type = validation.get_validation_type
            if validation_type != validation_type_initial:
                raise TypeError("type of validation instances is not consistent")
                # do I need to write a return statement or will the error account for this
        return validation_type_initial