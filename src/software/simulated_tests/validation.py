import pytest


import software.geom.geometry as tbots_geom
from proto.validation_pb2 import (
    ValidationGeometry,
    ValidationProto,
    ValidationProtoSet,
    ValidationStatus,
    ValidationType,
)


class Validation(object):

    """A validation function"""

    def get_validation_type(self, vision) -> ValidationType:
        raise NotImplementedError("get_validation_type is not implemented")

    def get_validation_geometry(self, vision) -> ValidationGeometry:
        raise NotImplementedError("get_validation_geometry is not implemented")


def createValidationTypes(validation_class):
    """Given a Validation implementation that returns ValidationStatus.PASSING
    when true and ValidationStatus.FAILING when false, create the 4 validation
    types with different visualization/passing/failing properties.
     
                              ┌───────────────────────┐      ┌─────────────────┐
                              │                       │──────► EventuallyTrue  │
                              │                       │      └─────────────────┘
                              │                       │
                              │                       │      ┌─────────────────┐
                              │                       ├──────► EventuallyFalse │
     ┌─────────────────┐      │ createValidationTypes │      └─────────────────┘
     │    Validation   ├──────►                       │
     └─────────────────┘      │                       │      ┌─────────────────┐
                              │                       ├──────►   AlwaysTrue    │
                              │                       │      └─────────────────┘
                              │                       │
                              │                       │      ┌─────────────────┐
                              │                       ├──────►   AlwaysFalse   │
                              └───────────────────────┘      └─────────────────┘

    EventuallyTrue: Has to be true before the end of the test. A series of these
                    validations
    EventuallyFalse: Has to be false before the end of the test. Inverts the
                     validation so passing becomes failing (and vice versa)
    AlwaysTrue: Has to be true for the duration of the test.
    AlwaysFalse: Has to be false for the duration of the test.

    NOTE: EventuallyFalse validation is a flipped EventuallyTrue validation
          AlwaysTrue validation checks the same condition, but will stay 
          validation 

    :param eventually_true: A validation function that is eventually_true
    :returns: EventuallyTrueValidation, EventuallyFalseValidation,
              AlwaysTrueValidation, AlwaysFalseValidation
    """

    def constructor(self, *args, **kwargs):
        """The validations will be composed of the input validation type

        :param args/kwargs: Pass through to the validation_class

        """
        self.validation = validation_class(*args, **kwargs)

    def flip_validation(self, vision):
        """Flip the validation status

        :param vision: The vision msg to validate on

        """

        return {
            ValidationStatus.FAILING: ValidationStatus.PASSING,
            ValidationStatus.PASSING: ValidationStatus.FAILING,
        }[self.validation.get_validation_status(vision)]

    eventually_true = type(
        "EventuallyTrueValidation",
        (Validation,),
        {
            "__init__": constructor,
            "__repr__": lambda self: "EventuallyTrueValidation: "
            + repr(self.validation_class),
            "get_validation_type": lambda self: ValidationType.EVENTUALLY,
            "get_validation_status": lambda self, vision: self.validation.get_validation_status(
                vision
            ),
            "get_validation_geometry": lambda self, vision: self.validation.get_validation_geometry(
                vision
            ),
        },
    )

    eventually_false = type(
        "EventuallyFalseValidation",
        (Validation,),
        {
            "__init__": constructor,
            "__repr__": lambda self: "EventuallyFalseValidation: "
            + repr(self.validation),
            "get_validation_type": lambda self: ValidationType.EVENTUALLY,
            "get_validation_status": lambda self, vision: flip_validation(self, vision),
            "get_validation_geometry": lambda self, vision: self.validation.get_validation_geometry(
                vision
            ),
        },
    )

    always_true = type(
        "AlwaysTrueValidation",
        (Validation,),
        {
            "__init__": constructor,
            "__repr__": lambda self: "AlwaysTrueValidation: " + repr(self.validation),
            "get_validation_type": lambda self: ValidationType.ALWAYS,
            "get_validation_status": lambda self, vision: self.validation.get_validation_status(
                vision
            ),
            "get_validation_geometry": lambda self, vision: self.validation.get_validation_geometry(
                vision
            ),
        },
    )

    always_false = type(
        "AlwaysFalseValidation",
        (Validation,),
        {
            "__init__": constructor,
            "__repr__": lambda self: "AlwaysFalseValidation: " + repr(self.validation),
            "get_validation_type": lambda self: ValidationType.ALWAYS,
            "get_validation_status": lambda self, vision: flip_validation(self, vision),
            "get_validation_geometry": lambda self, vision: self.validation.get_validation_geometry(
                vision
            ),
        },
    )

    return eventually_true, eventually_false, always_true, always_false


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

    :returns: ValidationProto

    """

    # Proto that stores validation geometry and validation status
    validation_proto_set = ValidationProtoSet()

    def create_validation_proto_helper(validation):
        """Helper function call get_validation_status and add a
        validation_proto to validation_proto_set.

        :param validation: The validation to run
        :return: Return status

        """
        # Stores the validation result
        validation_proto = ValidationProto()

        # Get result
        status = validation.get_validation_status(vision)

        # Create validation proto
        validation_proto.status = status
        validation_proto.failure_msg = str(validation) + " failed"
        validation_proto.validation_type = validation.get_validation_type()
        validation_proto.geometry.CopyFrom(validation.get_validation_geometry(vision))

        validation_proto_set.validations.append(validation_proto)

        return status

    # Validate the eventually validations. Eventually valids
    for validation_sequence in eventually_validation_sequence_set:
        for validation in validation_sequence:

            # Add to validation_proto_set and get status
            status = create_validation_proto_helper(validation)

            # If the current validation is pending, we don't care about
            # the next one. Keep evaluating until this one passes.
            if status == ValidationStatus.FAILING:
                break

            # If the validation has passed, continue
            # this line is not needed, but just added to be explicit
            if status == ValidationStatus.PASSING:
                continue

    # Validate the always validations. We need to look at all of them
    for validation_sequence in always_validation_sequence_set:
        for validation in validation_sequence:

            create_validation_proto_helper(validation)

    return validation_proto_set


def check_always_validation(validation_proto_set):
    """Check always validations and make sure its always true

    :param validation_proto_set: Validation proto set
    :raises: AssertionError

    """
    for validation_proto in validation_proto_set.validations:
        if validation_proto.validation_type == ValidationType.ALWAYS:
            if validation_proto.status == ValidationStatus.FAILING:
                raise AssertionError(validation_proto.failure_msg)


def check_eventually_validation(validation_proto_set):
    """Check eventually validations and make sure they are all true

    :param validation_proto_set: Validation proto set
    :raises: AssertionError

    """
    for validation_proto in validation_proto_set.validations:
        if validation_proto.validation_type == ValidationType.EVENTUALLY:
            if validation_proto.status == ValidationStatus.FAILING:
                raise AssertionError(validation_proto.failure_msg)


def create_validation_geometry(geometry=[]) -> ValidationGeometry:
    """Given a list of (vectors, polygons, circles), creates a ValidationGeometry
    proto containing the protobuf representations.

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
