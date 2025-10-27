from proto.validation_pb2 import *
from software.simulated_tests.validation import (
    Validation,
)
from typing import override


class OrValidation(Validation):
    def __init__(self, validations):
        """An OR extension to the validation function"""
        assert len(validations) > 0
        validation_type_initial = validations[0].get_validation_type()
        for validation in validations:
            validation_type = validation.get_validation_type()
            if validation_type != validation_type_initial:
                raise TypeError("Type of validation instances is not consistent")
        self.validations = validations

    @override
    def get_validation_status(self, world):
        for validation in self.validations:
            if validation.get_validation_status(world) == ValidationStatus.PASSING:
                return ValidationStatus.PASSING
        return ValidationStatus.FAILING

    @override
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

    @override
    def get_validation_type(self, world):
        return self.validations[0].get_validation_type(world)
