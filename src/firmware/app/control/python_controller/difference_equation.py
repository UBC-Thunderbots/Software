#!/usr/bin/env python
"""
This file contains the implementation of the DifferenceEquation and DifferenceEquationSimulator class 
which generates a difference equation based on an input discrete transfer function.

"""

import numpy as np
import control as ct
from control.matlab import *
import matplotlib.pyplot as plt
import functools
import collections
from typing import List


class DifferenceEquation:
    def __init__(self, discrete_tf: ct.TransferFunction):
        """ 
        Creates a difference equation based on the input discrete domain transfer function

        :param discrete_tf The discrete transfer function (Z domain) representing the system
        """
        # Copy the numerator and denominator of the input TF
        numerator_coefficients = discrete_tf.num[0][0]
        denominator_coefficients = discrete_tf.den[0][0]

        # Store the order of the numerator and denominator for calculation later
        self.__system_order = len(denominator_coefficients)
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
        # so we will remove it here so it has no contribution
        del self.__output_coefficients[0]

    def calculate_output(
        self, output_history: List[float], input_history: List[float]
    ) -> float:
        """ 
        Calculates the output of the difference equation given the list of previous(and current) outputs
        and inputs to the system
        
        :param output_history The last M outputs, where M is the order of the original TF's denominator
        aka self.__system_order
        
        :param input_history The last N inputs, where N is the order of the original TF's numerator
        aka self.__numerator_order
        """

        # Calculate the effect of the input history has on the output
        input_response = 0
        history_index_shift = self.__system_order - self.__numerator_order
        for i in range(history_index_shift, self.__system_order):
            input_response += (
                input_history[i] * self.__input_coefficients[i - history_index_shift]
            )

        # Calculate the effect of the output history has on the output
        output_response = [
            coefficient * input_value
            for coefficient, input_value in zip(
                self.__output_coefficients, output_history
            )
        ]

        return input_response - functools.reduce(lambda a, b: a + b, output_response)

    def get_system_order(self):
        """
        Returns the order of the system (denominator of TF) polynomial
        """
        return self.__system_order

    def get_input_order(self):
        """
        Returns the order of the input(numerator of TF) polynomial
        """
        return self.__numerator_order

    def get_input_coefficients(self):
        """
        Returns the Coefficients of the input polynomial from the least negative Z 
        exponent to the most negative. (Z^0, Z^-1, Z^-2...)
        """
        return self.__input_coefficients

    def get_output_coefficients(self):
        """
        Returns the Coefficients of the output polynomial from the least negative Z 
        exponent to the most negative. (Z^0, Z^-1, Z^-2...)
        """
        return self.__output_coefficients


class DifferenceEquationSimulator:
    def __init__(self, difference_equation: DifferenceEquation):
        """ 
        Creates a difference equation simulator based on the input difference equation transfer function
f
        :param difference_equation The difference equation transfer function (Z domain) representing the system
        """

        # Create the circular buffers that are sized to contain all the needed information
        # required to run the difference equation
        self.__input_buffer = collections.deque(
            maxlen=difference_equation.get_system_order()
        )
        self.__output_buffer = collections.deque(
            maxlen=difference_equation.get_system_order()
        )

        # Fill each buffer with zeros as initialization
        for i in range(0, difference_equation.get_system_order()):
            self.__input_buffer.append(0)
            self.__output_buffer.append(0)

        self.difference_equation = difference_equation
        self.full_output_history = []

    def tick(self, input_value: float):
        """ 
        Calculates the output of the difference equation based on the parameter input signal and the existing input
        and output history

        :param input_value The current input into the system
        """
        # Add the input to the buffer
        self.__input_buffer.appendleft(input_value)

        # Calculate the output of the difference equation
        output = self.difference_equation.calculate_output(
            self.__output_buffer, self.__input_buffer
        )

        self.__output_buffer.appendleft(output)
        self.full_output_history.append(output)

        return output

    def run_for_ticks(self, number_of_ticks: int, input_value: float):
        """ 
        Calculates the output of the difference equation based on the parameter input signal and the existing input
        and output history for the specified number of cycles.
        
        The input is assumed to be constant.
        
        :param number_of_ticks The number of iterations to simulate with the given input

        :param input_value The current input into the system
        """

        for i in range(0, number_of_ticks):
            self.tick(input_value)
