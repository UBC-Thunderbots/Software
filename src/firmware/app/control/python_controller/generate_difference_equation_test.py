#!/usr/bin/env python
"""
This file contains the unit tests for the generate_difference_equation.py file

"""

import sys
import control as ct
from control.matlab import *
import unittest
import generate_difference_equation as gde


class TestDifferenceEquationCoefficientGeneration(unittest.TestCase):
    def test_generate_string(self):
        """
        Test that the generate function correctly formats CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS
        with the provided transfer function
        """

        expected = """
#define WHEEL_SPEED_CONTROLLER_TICK_TIME 0.1;
#define GENERATED_NUM_WHEEL_SPEED_COEFFICIENTS 2;
#define GENERATED_NUM_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS 3;
float GENERATED_WHEEL_SPEED_COEFFICIENTS[GENERATED_NUM_WHEEL_SPEED_COEFFICIENTS] =
{5, 4};
float GENERATED_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS[GENERATED_NUM_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS] =
{3, 2, 1};
"""

        numerator = [5, 4]
        denominator = [3, 2, 1]
        interpolation_period = 0.1

        transfer_function = tf(numerator, denominator, interpolation_period)
        difference_equation_coefficients = gde.generate(transfer_function)

        # Remove whitespace to keep comparison consistent
        self.assertEqual(difference_equation_coefficients.strip(), expected.strip())


if __name__ == "__main__":
    unittest.main()
