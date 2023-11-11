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

    def get_validation_type(self, world): # do I need to return anything here or even include this at all
        validation_type_initial = ValidationType

        for validation in self.validation:
            validation_type = validation.get_validation_type
            if validation_type != validation_type_initial:
                raise TypeError("type of validation instances is not consistent")
            
        return validation_type_initial


    def get_validation_geometry(self, world):

        validation_geometry = ValidationGeometry()

        for validation in self.validation:
            validation.get_validation_geometry(world) # why is this needed
            validation_geometry.polygons += validation.polygons
            validation_geometry.circles += validation.circles
            validation_geometry.vectors += validation.vectors
            validation_geometry.segments += validation.segments

        return validation_geometry