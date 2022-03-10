import logging

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
