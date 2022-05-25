#!/usr/bin/env python
"""
This file contains the unit tests for the DifferenceEquation class

"""

import numpy as np
import control as ct
from control.matlab import *
import unittest
import difference_equation as de
import collections


# TODO As part of #1381: https://github.com/UBC-Thunderbots/Software/issues/1381
# these unit tests must be re-done using Pytest as the testing framework
class TestDifferenceEquation(unittest.TestCase):
    def test_simulator_tick(self):
        """
        Test that the difference equation simulator follows the correct output sequence
        as a human-checked sequence with constant input
        """

        numerator = [1, 1]
        denominator = [1, 1, 1]
        interpolation_period = 0.1
        input_value = 1

        transfer_function = tf(numerator, denominator, interpolation_period)
        difference_equation = de.DifferenceEquation(transfer_function)

        deSim = de.DifferenceEquationSimulator(difference_equation)

        deSim.tick(input_value)
        deSim.tick(input_value)
        deSim.tick(input_value)
        deSim.tick(input_value)
        deSim.tick(input_value)
        deSim.tick(input_value)

        self.assertEqual(deSim.full_output_history[0], 0)
        self.assertEqual(deSim.full_output_history[1], 1)
        self.assertEqual(deSim.full_output_history[2], 1)
        self.assertEqual(deSim.full_output_history[3], 0)
        self.assertEqual(deSim.full_output_history[4], 1)
        self.assertEqual(deSim.full_output_history[4], 1)

    def test_simulator_run_for_ticks(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function
        for a specified number of ticks
        """

        sample_time = 0.01  # [seconds]
        end_time = 20

        coefficient_1 = 0.01
        coefficient_2 = 0.001
        coefficient_3 = 1

        # Construct the Laplace operator
        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = (coefficient_3 * s ** 2 + coefficient_3 * coefficient_2 * s) / (
            coefficient_1 * s ** 2 + coefficient_2 * s + 1.5
        )

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")

        difference_equation = de.DifferenceEquation(discrete_tf)

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        # Create and run the difference equation simulator for the same number
        # of ticks as the control toolbox discrete controller
        deSim = de.DifferenceEquationSimulator(difference_equation)
        deSim.run_for_ticks(num_points, step_input)

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], deSim.full_output_history[i], 4)

    def test_difference_equation_init(self):
        """
        Tests that the difference equation initialization function creates and normalized the input discrete TF
        properly 
        """
        numerator = [1, 0]
        denominator = [1, 1, 1]
        interpolation_period = 0.1

        transfer_function = ct.tf(numerator, denominator, interpolation_period)

        difference_equation = de.DifferenceEquation(transfer_function)

        self.assertEqual(difference_equation.get_system_order(), len(denominator))
        self.assertEqual(difference_equation.get_input_order(), len(numerator))

        for i in range(0, len(numerator)):
            self.assertEqual(
                difference_equation.get_input_coefficients()[i], numerator[i]
            )

        # Delete denominator first element to match up array lengths
        del denominator[0]

        for i in range(1, len(denominator)):
            self.assertEqual(
                difference_equation.get_output_coefficients()[i], denominator[i]
            )

        numerator = [1, 0]
        denominator = [2, 1, 1]
        interpolation_period = 0.1

        transfer_function = ct.tf(numerator, denominator, interpolation_period)

        difference_equation = de.DifferenceEquation(transfer_function)

        self.assertEqual(difference_equation.get_input_coefficients()[0], 0.5)
        self.assertEqual(difference_equation.get_input_coefficients()[1], 0)

        self.assertEqual(difference_equation.get_output_coefficients()[0], 0.5)
        self.assertEqual(difference_equation.get_output_coefficients()[1], 0.5)

    def test_difference_equation_output(self):
        """
        Test that the difference equation follows the correct output sequence
        as a human-checked sequence with constant input
        """

        numerator = [1, 1]
        denominator = [1, 1, 1]
        interpolation_period = 0.1
        input_value = 1

        # Set up circular buffers for input/output management
        inputs = collections.deque(maxlen=len(denominator))
        outputs = collections.deque(maxlen=len(denominator))

        for i in range(0, len(denominator)):
            inputs.append(0)
        for i in range(0, len(denominator)):
            outputs.append(0)

        transfer_function = tf(numerator, denominator, interpolation_period)
        difference_equation = de.DifferenceEquation(transfer_function)
        inputs.appendleft(input_value)

        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))
        self.assertEqual(outputs[0], 0)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], 1)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], 1)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], 0)

        inputs.appendleft(input_value)
        outputs.appendleft(difference_equation.calculate_output(outputs, inputs))

        self.assertEqual(outputs[0], 1)

    def test_output_of_difference_equation_against_discrete_tf_first_order(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function (1st order)
        using the python control toolbox
        """

        sample_time = 0.1  # [seconds]
        end_time = 5

        coefficient_1 = 0.001
        coefficient_2 = 0.001
        coefficient_3 = 1 / 1000

        # Construct the Laplace operator
        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = coefficient_3 / (coefficient_1 * s + coefficient_2)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")

        difference_equation = de.DifferenceEquation(discrete_tf)

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        # Set up circular buffers for input/output management
        inputs = collections.deque(maxlen=difference_equation.get_system_order())
        outputs = collections.deque(maxlen=difference_equation.get_system_order())

        full_output_history = []

        for i in range(0, difference_equation.get_system_order()):
            inputs.append(0)
        for i in range(0, difference_equation.get_system_order()):
            outputs.append(0)

        for i in range(0, num_points):
            inputs.appendleft(step_input)
            output = difference_equation.calculate_output(outputs, inputs)
            outputs.appendleft(output)
            full_output_history.append(output)

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], full_output_history[i], 4)

    def test_output_of_difference_equation_against_discrete_tf_second_order(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function (2nd order)
        using the python control toolbox
        """

        sample_time = 0.01  # [seconds]
        end_time = 20

        coefficient_1 = 0.01
        coefficient_2 = 0.001
        coefficient_3 = 1

        # Construct the Laplace operator
        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = coefficient_3 / (
            coefficient_1 * s ** 2 + coefficient_2 * s + 1.5
        )

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")

        difference_equation = de.DifferenceEquation(discrete_tf)

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        # Set up circular buffers for input/output management
        inputs = collections.deque(maxlen=difference_equation.get_system_order())
        outputs = collections.deque(maxlen=difference_equation.get_system_order())

        full_output_history = []

        for i in range(0, difference_equation.get_system_order()):
            inputs.append(0)
        for i in range(0, difference_equation.get_system_order()):
            outputs.append(0)

        for i in range(0, num_points):
            inputs.appendleft(step_input)
            output = difference_equation.calculate_output(outputs, inputs)
            outputs.appendleft(output)
            full_output_history.append(output)

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], full_output_history[i], 4)

    def test_output_of_difference_equation_against_discrete_tf_numerator_and_denominator_same_order(
        self,
    ):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function
        for a system where the numerator and denominator are equal order
        """

        sample_time = 0.01  # [seconds]
        end_time = 20

        coefficient_1 = 0.01
        coefficient_2 = 0.001
        coefficient_3 = 1

        # Construct the Laplace operator
        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = (coefficient_3 * s ** 2 + coefficient_3 * coefficient_2 * s) / (
            coefficient_1 * s ** 2 + coefficient_2 * s + 1.5
        )

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")

        difference_equation = de.DifferenceEquation(discrete_tf)

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        # Set up circular buffers for input/output management
        inputs = collections.deque(maxlen=difference_equation.get_system_order())
        outputs = collections.deque(maxlen=difference_equation.get_system_order())

        full_output_history = []

        for i in range(0, difference_equation.get_system_order()):
            inputs.append(0)
        for i in range(0, difference_equation.get_system_order()):
            outputs.append(0)

        for i in range(0, num_points):
            inputs.appendleft(step_input)
            output = difference_equation.calculate_output(outputs, inputs)
            outputs.appendleft(output)
            full_output_history.append(output)

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], full_output_history[i], 4)

    def test_output_of_difference_equation_against_discrete_tf_high_order(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function
        for a system where the system has high order
        """

        sample_time = 0.10  # [seconds]
        end_time = 10

        coefficient_1 = 0.01
        coefficient_2 = 0.001
        coefficient_3 = 1

        # Construct the Laplace operator
        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = (coefficient_3 * s) / (
            coefficient_1 * s ** 3 + coefficient_2 * s + 1.5
        )

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")

        difference_equation = de.DifferenceEquation(discrete_tf)

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        # Set up circular buffers for input/output management
        inputs = collections.deque(maxlen=difference_equation.get_system_order())
        outputs = collections.deque(maxlen=difference_equation.get_system_order())

        full_output_history = []

        for i in range(0, difference_equation.get_system_order()):
            inputs.append(0)
        for i in range(0, difference_equation.get_system_order()):
            outputs.append(0)

        for i in range(0, num_points):
            inputs.appendleft(step_input)
            output = difference_equation.calculate_output(outputs, inputs)
            outputs.appendleft(output)
            full_output_history.append(output)

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], full_output_history[i], 1)


if __name__ == "__main__":
    unittest.main()
