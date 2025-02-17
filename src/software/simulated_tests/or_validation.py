from proto.validation_pb2 import *
from software.simulated_tests.validation import (
    Validation,
)


class OrValidation(Validation):
    def __init__(self, validations):
        """An or extension to the validation function"""
        self.validations = validations

    def get_validation_status(self, world):
        for validation in self.validations:
            if validation.get_validation_status(world) == ValidationStatus.PASSING:
                print(f"OR Validation PASSING {validation}")
                return ValidationStatus.PASSING
        print(f"OR Validation Failing: {validation}")
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
        return ValidationType
