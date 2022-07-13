import pytest


import software.python_bindings as tbots
from proto.validation_pb2 import *


class Validation(object):

    """A validation function"""

    def get_validation_status(self, world) -> ValidationStatus:
        raise NotImplementedError("get_validation_status is not implemented")

    def get_validation_type(self, world) -> ValidationType:
        raise NotImplementedError("get_validation_type is not implemented")

    def get_validation_geometry(self, world) -> ValidationGeometry:
        raise NotImplementedError("get_validation_geometry is not implemented")

    def __repr__(self):
        return "String representation of validation not implemented"


def create_validation_types(validation_class):
    """Given a Validation implementation that returns ValidationStatus.PASSING
    when true and ValidationStatus.FAILING when false, create the 4 validation
    types with different visualization/passing/failing properties (described below)
     
                              ┌───────────────────────┐      ┌─────────────────┐
                              │                       │──────► EventuallyTrue  │
                              │                       │      └─────────────────┘
                              │                       │
                              │                       │      ┌─────────────────┐
                              │                       ├──────► EventuallyFalse │
     ┌─────────────────┐      │                       │      └─────────────────┘
     │    Validation   ├──────►create_validation_types│
     └─────────────────┘      │                       │      ┌─────────────────┐
                              │                       ├──────►   AlwaysTrue    │
                              │                       │      └─────────────────┘
                              │                       │
                              │                       │      ┌─────────────────┐
                              │                       ├──────►   AlwaysFalse   │
                              └───────────────────────┘      └─────────────────┘

    EventuallyTrue: Has to be true before the end of the test.
    EventuallyFalse: Has to be false before the end of the test. Inverts the
                     validation so passing becomes failing (and vice versa)
    AlwaysTrue: Has to be true for the duration of the test.
    AlwaysFalse: Has to be false for the duration of the test.

    NOTE: EventuallyFalse validation is a flipped EventuallyTrue validation
          AlwaysTrue validation checks the same condition, but needs to be
          always true. AlwaysFalse is the flipped AlwaysTrue

    :param eventually_true: A validation function that is eventually_true
    :returns: EventuallyTrueValidation, EventuallyFalseValidation,
              AlwaysTrueValidation, AlwaysFalseValidation
    """

    def constructor(self, *args, **kwargs):
        """The 4 validation outputs will be composed of the input validation

        :param args/kwargs: Pass through to the validation_class

        """
        self.validation = validation_class(*args, **kwargs)

    def flip_validation(self, world):
        """Flip the validation status

        :param world: The world msg to validate on

        """

        return {
            ValidationStatus.FAILING: ValidationStatus.PASSING,
            ValidationStatus.PASSING: ValidationStatus.FAILING,
        }[self.validation.get_validation_status(world)]

    # Generate the types: specifically, all Eventually validations will return
    # EVENTUALLY when get_validation_type is called, and all Always validations
    # will return ALWAYS. get_validation_status is inverted for the _False types.
    # We simply pass the validation_geometry from the validation object through.
    common = {
        "__init__": constructor,
        "get_validation_geometry": lambda self, world: self.validation.get_validation_geometry(
            world
        ),
    }

    eventually_true = type(
        "EventuallyTrueValidation",
        (Validation,),
        {
            **common,
            "__repr__": lambda self: "EventuallyTrueValidation: "
            + repr(self.validation),
            "get_validation_type": lambda self: ValidationType.EVENTUALLY,
            "get_validation_status": lambda self, world: self.validation.get_validation_status(
                world
            ),
        },
    )

    eventually_false = type(
        "EventuallyFalseValidation",
        (Validation,),
        {
            **common,
            "__repr__": lambda self: "EventuallyFalseValidation: "
            + repr(self.validation),
            "get_validation_type": lambda self: ValidationType.EVENTUALLY,
            "get_validation_status": lambda self, world: flip_validation(self, world),
        },
    )

    always_true = type(
        "AlwaysTrueValidation",
        (Validation,),
        {
            **common,
            "__repr__": lambda self: "AlwaysTrueValidation: " + repr(self.validation),
            "get_validation_type": lambda self: ValidationType.ALWAYS,
            "get_validation_status": lambda self, world: self.validation.get_validation_status(
                world
            ),
        },
    )

    always_false = type(
        "AlwaysFalseValidation",
        (Validation,),
        {
            **common,
            "__repr__": lambda self: "AlwaysFalseValidation: " + repr(self.validation),
            "get_validation_type": lambda self: ValidationType.ALWAYS,
            "get_validation_status": lambda self, world: flip_validation(self, world),
        },
    )

    return eventually_true, eventually_false, always_true, always_false


def run_validation_sequence_sets(
    world, eventually_validation_sequence_set, always_validation_sequence_set
):
    """Given both eventually and always validation sequence sets, (and world)
    run validation and aggregate the results in a validation proto set.

    :raises AssertionError: If the test fails
    :param world: World to validate with
    :param eventually_validation_sequence_set:
            A collection of sequences of eventually validations to validate.
    :param always_validation_sequence_set:
            A collection of sequences of always validations to validate.

    :returns: Eventually ValidationProtoSet, Always ValidationProtoSet

    """

    # Proto that stores validation geometry and validation status of
    # all validations passed in
    always_validation_proto_set = ValidationProtoSet()
    always_validation_proto_set.validation_type = ValidationType.ALWAYS
    eventually_validation_proto_set = ValidationProtoSet()
    eventually_validation_proto_set.validation_type = ValidationType.EVENTUALLY

    def create_validation_proto_helper(validation_proto_set, validation):
        """Helper function that computes the status and creates a
        validation_proto, and updates it in the validation_proto_set.

        :param validation_proto_set: The validation proto set to add to
        :param validation: The validation to put into the proto

        """
        # Stores the validation result
        validation_proto = ValidationProto()

        # Get status
        status = validation.get_validation_status(world)

        # Create validation proto
        validation_proto.status = status
        validation_proto.failure_msg = str(validation) + " failed"
        validation_proto.geometry.CopyFrom(validation.get_validation_geometry(world))

        validation_proto_set.validations.append(validation_proto)

        return status

    # Validate the eventually validations. Eventually valids
    for validation_sequence in list(eventually_validation_sequence_set):
        for validation in validation_sequence:

            # Add to validation_proto_set and get status
            status = create_validation_proto_helper(
                eventually_validation_proto_set, validation
            )

            # If the current validation is failing, we don't care about
            # the next one. Keep evaluating until this one passes.
            if status == ValidationStatus.FAILING:
                break

            # If the validation has passed, remove it from the set.
            if status == ValidationStatus.PASSING:
                validation_sequence.remove(validation)
                continue

    # Validate the always validations. We need to look at all of them
    for validation_sequence in always_validation_sequence_set:
        for validation in validation_sequence:
            create_validation_proto_helper(always_validation_proto_set, validation)

    return eventually_validation_proto_set, always_validation_proto_set


def check_validation(validation_proto_set):
    """Check validation and make sure its always true

    :param validation_proto_set: Validation proto set
    :raises: AssertionError

    """
    for validation_proto in validation_proto_set.validations:
        if validation_proto.status == ValidationStatus.FAILING:
            raise AssertionError(validation_proto.failure_msg)


def create_validation_geometry(geometry=[]) -> ValidationGeometry:
    """Creates a ValidationGeometry which is a visual representation of the
    validation to be rendered as either green (PASSING) or red (FAILING)

    Given a list of (vectors, polygons, circles), creates a ValidationGeometry
    proto containing the protobuf representations.

    :param geometry: A list of geom
    :returns: ValidationGeometry

    """

    validation_geometry = ValidationGeometry()

    CREATE_PROTO_DISPATCH = {
        tbots.Vector.__name__: tbots.createVectorProto,
        tbots.Polygon.__name__: tbots.createPolygonProto,
        tbots.Rectangle.__name__: tbots.createPolygonProto,
        tbots.Circle.__name__: tbots.createCircleProto,
    }

    ADD_TO_VALIDATION_GEOMETRY_DISPATCH = {
        tbots.Vector.__name__: validation_geometry.vectors.append,
        tbots.Polygon.__name__: validation_geometry.polygons.append,
        tbots.Rectangle.__name__: validation_geometry.polygons.append,
        tbots.Circle.__name__: validation_geometry.circles.append,
    }

    for geom in geometry:
        ADD_TO_VALIDATION_GEOMETRY_DISPATCH[type(geom).__name__](
            CREATE_PROTO_DISPATCH[type(geom).__name__](geom)
        )

    return validation_geometry


def contains_failure(validation_proto_set: ValidationProtoSet):
    for validation in validation_proto_set.validations:
        if validation.status == ValidationStatus.FAILING:
            return True

    return False
