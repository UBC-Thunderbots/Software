import pytest

import software.python_bindings as tbots
from proto.validation_pb2 import *
from software.simulated_tests.validation import (
    Validation,
    create_validation_geometry,
    create_validation_types,
)


class OrValidation(Validation):
    def __init__(self, validations):
        """An or extension to the validation function"""
        self.validations = validations

    def get_validation_status(self, world):
        for validation in self.validations:
            if validation.get_validation_status(world) == ValidationStatus.PASSING:
                return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    def get_validation_geometry(self, world):

        validation_geometry = ValidationGeometry()

        for validation in self.validations:
            individual_geometry = validation.get_validation_geometry(world)
            for polygon in individual_geometry.polygons:
                validation_geometry.polygons.append(polygon)
            for circles in individual_geometry.circles:
                validation_geometry.circles.append(circles)
            for vectors in individual_geometry.vectors:
                validation_geometry.vectors.append(vectors)
            for segments in individual_geometry.segments:
                validation_geometry.segments.append(segments)

        return validation_geometry

    def get_validation_type(self, world):
        validation_type_initial = self.validations[0].get_validation_type

        for validation in self.validations:
            validation_type = validation.get_validation_type
            if validation_type != validation_type_initial:
                raise TypeError("type of validation instances is not consistent")
        return validation_type_initial
