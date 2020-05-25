#!/usr/bin/env python
"""
This file contains the implementation of the DifferenceEquation class 
which generates a difference equation based on an input discrete transfer function.

"""

import numpy as np
import control as ct
from control.matlab import *
import matplotlib.pyplot as plt
import functools


class Difference_Equation:
    def __init__(self, discrete_tf: ct.TransferFunction):
        """ Creates a difference equation based on the input discrete domain transfer function

        :param discrete_tf The discrete transfer function (Z domain) representing the system
        """
        # Copy the numerator and denominator of the input TF
        numerator_coefficients = discrete_tf.num[0][0]
        denominator_coefficients = discrete_tf.den[0][0]

        # Store the order of the numerator and denominator for calculation later
        self.__denominator_order = len(denominator_coefficients)
        self.__numerator_order = len(numerator_coefficients)

        # Normalize by the coefficient of the highest order denominator term
        # This is done so that the output of the difference equation is not scaled
        self.__input_coefficients = [
            coeff / denominator_coefficients[0] for coeff in numerator_coefficients
        ]
        self.__output_coefficients = [
            coeff / denominator_coefficients[0] for coeff in denominator_coefficients
        ]

        # The first element in the output coefficient list corresponds to the system output for a given input,
        # so we will 'remove' it here by setting it's value to zero - and therefor no contribution
        self.__output_coefficients[0] = 0

    def calculate_output(self, output_history, input_history):
        """ Calculated the output of the difference equation given the list of previous(and current) outputs
        and inputs to the system

        :param input_history The last N inputs, where N is the order of the original TF's numerator
        aka self.__numerator_order

        :param output_history The last M outputs, where M is the order of the original TF's denominator
        aka self.__denominator_order
        """

        # Use the input history and multiply with the input coefficients
        # TODO this section here doesn't account for how the index values of the coefficients
        # and the history element you would like to access to not line up
        # Solutions: Maybe add blank spaces to the beggining of the arrays
        # or perhaps do some shifting by a constant difference between orders
        # before doing the math
        input_response = 0
        for i in range(
            self.__denominator_order - self.__numerator_order, self.__denominator_order
        ):
            input_response += input_history[i] * self.__input_coefficients[i]

        # Use the output history and multiply with the output coefficients
        output_response = [
            coefficient * input_value
            for coefficient, input_value in zip(
                self.__output_coefficients, output_history
            )
        ]

        return input_response - functools.reduce(lambda a, b: a + b, output_response)

    def get_input_order(self):
        return self.__numerator_order

    def get_output_order(self):
        return self.__denominator_order

    def get_input_coefficients(self):
        return self.__input_coefficients

    def get_output_coefficients(self):
        return self.__output_coefficients


class DifferenceEquation:
    def __init__(
        self, tf_numerator_coefficients: list, tf_denominator_coefficients: list,
    ):

        """ Creates a difference equation based on the input discrete domain transfer function

        :list tf_numerator_coefficients: The coefficients of the numerator of the discrete transfer function
        these coefficients should be in decreasing order. Where tf_numerator_coefficients[0] is the largest

        :list tf_denominator_coefficients: The coefficients of the denominator of the discrete transfer function
        these coefficients should be in decreasing order. Where tf_denominator_coefficients[0] is the largest
        """
        self.__numerator_coefficients = tf_numerator_coefficients
        self.__denominator_coefficients = tf_denominator_coefficients

        # Store the order of the numerator and denominator for calculation later
        self.__denominator_order = len(self.__denominator_coefficients)
        self.__numerator_order = len(self.__numerator_coefficients)

        # Normalize by the coefficient of the highest order denominator term
        # This is done so that the output of the difference equation is not scaled
        for i in range(0, len(self.__numerator_coefficients)):
            self.__numerator_coefficients[i] /= self.__denominator_coefficients[0]
        for i in range(0, len(self.__denominator_coefficients)):
            self.__denominator_coefficients[i] /= self.__denominator_coefficients[0]

        # We must keep track of the system state and previous states
        self.__time_step_count = 0
        self.__previous_input = []
        self.__previous_output = []

    def tick(self, current_input: list):

        """ Ticks the difference equation to the next time step with the current input

        :float current_input: The current input to the discrete system
        """

        # Generate the systems response due to previous inputs
        effect_of_previous_input = 0

        # Loop through each coefficient of the numerator
        for i in range(0, self.__numerator_order):

            # Calculate how many input 'steps' back in time we must use for the given numerator coefficient
            index = self.__time_step_count + (
                self.__numerator_order - self.__denominator_order - i
            )

            # If we must go more time steps backwards than are recorded, the term has no effect on the output
            if index < 0 or len(self.__previous_input) < index + 1:
                effect_of_previous_input += 0
            else:
                effect_of_previous_input += (
                    self.__numerator_coefficients[i] * self.__previous_input[index]
                )

        # Generate the systems response due to previous output
        effect_of_previous_output = 0

        # Loop through each coefficient of the denominator other than the highest order term
        # as it represents the output of the tick
        for i in range(1, self.__denominator_order):

            # Calculate the number of time steps backwards to use
            index = self.__time_step_count - i

            # If we must go more time steps backwards than are recorded, the term has no effect on the output
            if index < 0 or len(self.__previous_output) < index + 1:
                effect_of_previous_output += 0
            else:
                effect_of_previous_output += (
                    self.__denominator_coefficients[i] * self.__previous_output[index]
                )

        # The new output of the system in the difference between the effect of the inputs and the effect of the outputs
        new_output = effect_of_previous_input - effect_of_previous_output

        # Append the current input to the history of inputs
        self.__previous_input.append(current_input)

        # Append the current output to the list of outputs
        self.__previous_output.append(new_output)

        # Increase the count of time steps
        self.__time_step_count += 1

        return new_output

    def get_output_history(self):
        return self.__previous_output

    def run_for_ticks(self, numer_of_ticks: int, input: float):
        for i in range(0, numer_of_ticks):
            self.tick(input)

    def get_timestep_count(self):
        return self.__time_step_count

    def get_input_coefficients(self):
        return self.__numerator_coefficients

    def get_output_coefficients(self):
        return self.__denominator_coefficients
