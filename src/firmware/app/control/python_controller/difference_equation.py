#!/usr/bin/env python
"""
This file contains the implementation of the DifferenceEquation class 
which generates a difference equation based on an input discrete transfer function.

"""

import numpy as np
import control as ct
from control.matlab import *
import matplotlib.pyplot as plt
import collections
import _collections_abc
import os


class DifferenceEquation:
    def __init__(
        self, tf_numerator_coefficients: list, tf_denominator_coefficients: list
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
        self.__numerator_coefficients = [coeff/self.__numerator_coefficients[0] for coeff in self.__numerator_coefficients]
        # for i in range(0, len(self.__numerator_coefficients)):
        #     self.__numerator_coefficients[i] /= self.__denominator_coefficients[0]
        self.__denominator_coefficients = [coeff/self.__numerator_coefficients[0] for coeff in self.__denominator_coefficients]
        # for i in range(0, len(self.__denominator_coefficients)):
        #     self.__denominator_coefficients[i] /= self.__denominator_coefficients[0]

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