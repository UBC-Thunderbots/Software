import pytest

import software.python_bindings as tbots
from proto.validation_pb2 import *
import validation

class OrValidation(validation):

    """An or extension to the validation function"""

    def get_validation_status(self, world):
        for validation in validation_set:
            status = validation.get_validation_status()
                if status:

