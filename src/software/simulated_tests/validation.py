import pytest

from typing import List, Set

import software.geom.geometry as tbots_geom
from proto.validation_pb2 import ValidationGeometry, ValidationProto, ValidationStatus


class Validation(object):

    """A validation function"""

    def get_validation_status(self, vision) -> ValidationStatus:
        raise NotImplementedError("get_validation_status is not implemented")

    def get_validation_geometry(self, vision) -> ValidationGeometry:
        raise NotImplementedError("get_validation_geometry is not implemented")

    def get_failure_message(self):
        raise NotImplementedError("get_failure_message is not implemented")


ValidationSequence = List[Validation]
ValidationSequenceSet = Set[ValidationSequence]


def flip_validation(get_validation_status_func):
    """Decorator to flip the validation function logic if
    should_flip is True.

    :param flip: If true, the result is flipped

    """

    def inner(self, vision):
        status = get_validation_status_func(self, vision)

        flipped = {
            ValidationStatus.FAILING: ValidationStatus.PASSING,
            ValidationStatus.PASSING: ValidationStatus.FAILING,
        }

        if self.flipped:
            return flipped[status]

        return status

    return inner


def create_validation_geometry(geometry=[]) -> ValidationGeometry:
    """Given a list of (vectors, polygons, circles), creates a ValidationGeometry
    proto containing the protobuf representations

    :param geometry: A list of geom
    :returns: ValidationGeometry

    """

    validation_geometry = ValidationGeometry()

    CREATE_PROTO_DISPATCH = {
        tbots_geom.Vector.__name__: tbots_geom.createVectorProto,
        tbots_geom.Polygon.__name__: tbots_geom.createPolygonProto,
        tbots_geom.Rectangle.__name__: tbots_geom.createPolygonProto,
        tbots_geom.Circle.__name__: tbots_geom.createCircleProto,
    }

    ADD_TO_VALIDATION_GEOMETRY_DISPATCH = {
        tbots_geom.Vector.__name__: validation_geometry.vectors.append,
        tbots_geom.Polygon.__name__: validation_geometry.polygons.append,
        tbots_geom.Rectangle.__name__: validation_geometry.polygons.append,
        tbots_geom.Circle.__name__: validation_geometry.circles.append,
    }

    for geom in geometry:
        ADD_TO_VALIDATION_GEOMETRY_DISPATCH[type(geom).__name__](
            CREATE_PROTO_DISPATCH[type(geom).__name__](geom)
        )

    return validation_geometry

def run_validation_sequence_sets(
    vision, eventually_validation_sequence_set, always_validation_sequence_set
):
    """Given both eventually and always validation sequence sets, (and vision)
    run validation and aggregate the results in a validation proto.

    :raises AssertionError: If the test fails
    :param vision: Vision to validate with
    :param eventually_validation_sequence_set:
            A collection of sequences of eventually validations to validate.
    :param always_validation_sequence_set:
            A collection of sequences of always validations to validate.

    :returns: ValidationProto, error_msg

    """

    # Proto that stores validation geometry and validation status
    validation_proto = ValidationProto()

    # Validate
    for validation_sequence in eventually_validation_sequence_set:

        # We only want to check the first
        for validation in validation_sequence:
            status = validation.get_validation_status(vision)

            validation_proto.status.append(status)
            validation_proto.geometry.append(validation.get_validation_geometry(vision))

            # If the current validation is pending, we don't care about
            # the next one. Keep evaluating until this one passes.
            if status == ValidationStatus.FAILING:
                break

            # If the validation has passed, continue
            # this line is not needed, but just added to be explicit
            if status == ValidationStatus.PASSING:
                continue

    return validation_proto
