#!/usr/bin/env python

import sys
import control as ct
from control.matlab import *

CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS = """
#define WHEEL_SPEED_CONTROLLER_TICK_TIME 1e7
#define GENERATED_NUM_WHEEL_SPEED_COEFFICIENTS {numerator_coefficients};
#define GENERATED_NUM_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS {denominator_coefficients};
float GENERATED_WHEEL_SPEED_COEFFICIENTS[GENERATED_NUM_WHEEL_SPEED_COEFFICIENTS] =
{{{numerator_length}}};
float GENERATED_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS[GENERATED_NUM_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS] =
{{{denominator_length}}};
"""

def generate(discrete_tf: ct.TransferFunction):
    # Copy the numerator and denominator of the input TF
    numerator_coefficients = discrete_tf.num[0][0]
    denominator_coefficients = discrete_tf.den[0][0]

    numerator_length = len(numerator_coefficients)
    denominator_length = len(denominator_coefficients)

    CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS.format(numerator_coefficients=numerator_coefficients, denominator_coefficients=denominator_coefficients, numerator_length=numerator_length, denominator_length=denominator_length)

#######################################################################
#                                MAIN                                 #
#######################################################################
if __name__ == "__main__":
    with open(sys.argv[-1], "w") as controller_difference_coefficient_generator:

        # header
        controller_difference_coefficient_generator.write(CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS)

    print("INFO: Generated Difference Equation Controller Coefficients")