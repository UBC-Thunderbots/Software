#!/usr/bin/env python

import sys
import control as ct
from control.matlab import *

CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS = """
#define WHEEL_SPEED_CONTROLLER_TICK_TIME 1e7
#define GENERATED_NUM_WHEEL_SPEED_COEFFICIENTS {numerator_length};
#define GENERATED_NUM_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS {denominator_length};
float GENERATED_WHEEL_SPEED_COEFFICIENTS[GENERATED_NUM_WHEEL_SPEED_COEFFICIENTS] =
{{{numerator_coefficients}}};
float GENERATED_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS[GENERATED_NUM_PREVIOUS_WHEEL_SPEED_COMMAND_COEFFICIENTS] =
{{{denominator_coefficients}}};
"""


def generate(discrete_tf: ct.TransferFunction):
    # Copy the numerator and denominator of the input TF
    # Convert from list to string to remove list brackets
    numerator_coefficients = str(discrete_tf.num[0][0].tolist())[1:-1]
    denominator_coefficients = str(discrete_tf.den[0][0].tolist())[1:-1]

    numerator_length = len(numerator_coefficients)
    denominator_length = len(denominator_coefficients)

    return CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS.format(
        numerator_coefficients=numerator_coefficients,
        denominator_coefficients=denominator_coefficients,
        numerator_length=numerator_length,
        denominator_length=denominator_length,
    )


#######################################################################
#                                MAIN                                 #
#######################################################################
if __name__ == "__main__":
    with open(sys.argv[-1], "w") as controller_coefficient_generator:
        # TODO hard-coded ATM, need to pass parameters from controller design file
        numerator = [1, 1]
        denominator = [1, 1, 1]
        interpolation_period = 0.1
        input_value = 1

        transfer_function = tf(numerator, denominator, interpolation_period)

        controller_coefficients = generate(transfer_function)

        # Generate difference equation coefficients
        controller_coefficient_generator.write(controller_coefficients)

    print("INFO: Generated Difference Equation Controller Coefficients")
