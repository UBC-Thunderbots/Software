#!/usr/bin/env python
"""
This file contains the implementation of the DifferenceEquation class which generates a difference equation based on an input discrete transfer function.

"""

import numpy as np
import control as ct
from control.matlab import *
import matplotlib.pyplot as plt

class DifferenceEquation:

    def __init__(self, tf_numerator_coefficients: list, tf_denominator_coefficients: list):

        """ Creates a difference equation based on the input discrete domain transfer function

        :list tf_numerator_coefficients: The coefficients of the numerator of the discrete transfer function
        :list tf_denominator_coefficients: The coefficients of the denominator of the discrete transfer function
        """
        self.numerator_coefficients = tf_numerator_coefficients
        self.denominator_coefficients = tf_denominator_coefficients

        # Store the order of the numerator and denominator for calculation later
        self.controller_rank = len(self.denominator_coefficients)
        self.input_coeffs_order = len(self.numerator_coefficients)

        # Normalize by the coefficient of the highest order denominator term
        # This is done so that the output of the difference equation is not scaled
        self.numerator_coefficients / self.denominator_coefficients[0]
        self.denominator_coefficients / self.denominator_coefficients[0]

        # We must keep track of the system state and previous states
        self.time_step_count = 0
        self.previous_input = []
        self.previous_output = []

    def tick(self, current_input):

        """ Ticks the difference equation to the next time step with the current input

        :float current_input: The current input to the discrete system
        """

        # Generate the systems response due to previous inputs
        effect_of_previous_input = 0

        # Loop through each coefficient of the numerator
        for i in range(0, self.input_coeffs_order):

            # Calculate how many input 'steps' back in time we must use for the given numerator coefficient
            index = self.time_step_count + (self.input_coeffs_order - self.controller_rank - i)

            # If we must go more time steps backwards than are recorded, the term has no effect on the output
            if(index < 0 or len(self.previous_input) < index+1):
                effect_of_previous_input += 0
            else:
                effect_of_previous_input += self.numerator_coefficients[i] * self.previous_input[index]

        # Generate the systems response due to previous output
        effect_of_previous_output = 0

        # Loop through each coefficient of the denominator other than the highest order term
        # as it represents the output of the tick
        for i in range(1, self.controller_rank):

            # Calculate the number of time steps backwards to use
            index = self.time_step_count - i

            # If we must go more time steps backwards than are recorded, the term has no effect on the output
            if(index <  0 or len(self.previous_output) < index+1):
                effect_of_previous_output += 0
            else:
                effect_of_previous_output += self.denominator_coefficients[i] * self.previous_output[index]

        # The new output of the system in the difference between the effect of the inputs and the effect of the outputs
        new_output = effect_of_previous_input - effect_of_previous_output

        # Append the current input to the history of inputs
        self.previous_input.append(current_input)

        # Append the current output to the list of outputs
        self.previous_output.append(new_output)

        # Increase the count of time steps
        self.time_step_count += 1
        
        return new_output