#!/usr/bin/env python

import sys
import control as ct
from control.matlab import *

# {} required to format string
# {{{}}} required to format string, initialize array, and 'escape' brackets
CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS = """
#define WHEEL_SPEED_CONTROLLER_TICK_TIME {tick_time};
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

    numerator_length = len(discrete_tf.num[0][0])
    denominator_length = len(discrete_tf.den[0][0])

    tick_time = discrete_tf.dt

    return CONTROLLER_DIFFERENCE_EQUATION_COEFFICIENTS.format(
        numerator_coefficients=numerator_coefficients,
        denominator_coefficients=denominator_coefficients,
        numerator_length=numerator_length,
        denominator_length=denominator_length,
        tick_time=tick_time,
    )


#######################################################################
#                                MAIN                                 #
#######################################################################
if __name__ == "__main__":
    with open(sys.argv[-1], "w") as controller_coefficient_generator:
        # TODO need to pass parameters from controller design file
        # https://github.com/UBC-Thunderbots/Software/issues/1520
        numerator = [1, 2]
        denominator = [3, 4, 5]
        interpolation_period = 0.1

        transfer_function = tf(numerator, denominator, interpolation_period)

        controller_coefficients = generate(transfer_function)

        # Generate difference equation coefficients
        controller_coefficient_generator.write(controller_coefficients)

    print("INFO: Generated Difference Equation Controller Coefficients")
