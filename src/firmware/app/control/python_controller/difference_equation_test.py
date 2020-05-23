#!/usr/bin/env python
"""
This file contains the unit tests for the DifferenceEquation class

"""

import numpy as np
import control as ct
from control.matlab import *
import unittest
import difference_equation as de


class TestDifferenceEquation(unittest.TestCase):
    def test_output(self):
        """
        Test that the difference equation follows the correct output sequence
        as a human-checked sequence with constant input
        """
        difference_equation = de.DifferenceEquation([1, 1], [1, 1, 1])

        output = difference_equation.tick(1)

        self.assertEqual(output, 0)

        output = difference_equation.tick(1)

        self.assertEqual(output, 1)

        output = difference_equation.tick(1)

        self.assertEqual(output, 1)

        output = difference_equation.tick(1)

        self.assertEqual(output, 0)

        output = difference_equation.tick(1)

        self.assertEqual(output, 1)

    def test_step_function_first_order(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function (1st order)
        using the python control toolbox
        """

        sample_time = 0.1  # [s]
        end_time = 5

        J = 0.001
        B = 0.001
        K = 1 / 1000

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K / (J * s + B)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")
        difference_equation = de.DifferenceEquation(
            discrete_tf.num[0][0], discrete_tf.den[0][0]
        )

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        for i in range(0, num_points):
            difference_equation.tick(step_input)

        system_response = difference_equation.get_output_history()

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], system_response[i], 4)

    def test_step_function_second_order(self):
        """
        This test checks the step response of the DifferenceEquation class vs the discrete transfer function (2nd order)
        using the python control toolbox
        """

        sample_time = 0.01  # [s]
        end_time = 20

        J = 0.01
        B = 0.001
        K = 1

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K / (J * s ** 2 + B * s + 1.5)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")
        difference_equation = de.DifferenceEquation(
            discrete_tf.num[0][0], discrete_tf.den[0][0]
        )

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        for i in range(0, num_points):
            difference_equation.tick(step_input)

        system_response = difference_equation.get_output_history()

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], system_response[i], 4)

    def test_run_for_ticks(self):
        """
        This test checks the run_for_ticks member function to ensure that the difference equation is ran for exactly the
        number of specified iterations
        """

        sample_time = 0.1  # [s]
        end_time = 20

        J = 0.01
        B = 0.001
        K = 1

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K / (J * s ** 2 + B * s + 1.5)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, "zoh")
        difference_equation = de.DifferenceEquation(
            discrete_tf.num[0][0], discrete_tf.den[0][0]
        )

        num_points = int(end_time / sample_time) + 1

        T = np.linspace(0, end_time, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        difference_equation.run_for_ticks(num_points, step_input)

        system_response = difference_equation.get_output_history()

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], system_response[i], 4)

        self.assertEqual(num_points, difference_equation.get_timestep_count())

    def test_get_time_step_count(self):
        """
        This test checks the get_time_step_count() member function to ensure it returns the exact numer of steps
        the difference equation has iterated through
        """

        denominator = [1, 2, 4]
        numerator = [1]

        difference_equation = de.DifferenceEquation(denominator, numerator);

        for i in range(0, 100):
            self.assertEqual(difference_equation.get_timestep_count() , i)
            difference_equation.tick(0)


if __name__ == "__main__":
    unittest.main()
