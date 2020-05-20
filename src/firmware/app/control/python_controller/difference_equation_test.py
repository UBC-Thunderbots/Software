#!/usr/bin/env python
"""
This file contains the unit tests for the DifferenceEquation class

"""

import numpy as np
import control as ct
from control.matlab import *
import matplotlib.pyplot as plt
import unittest
from firmware.app.control.python_controller.difference_equation import DifferenceEquation


class TestDifferenceEquation(unittest.TestCase):

    def test_step_function(self):

        sample_time = 0.0001  # [s]
        end_time = 5

        J = 0.001
        B = 0.001
        K = 1/1000

        s = ct.tf([1, 0], 1)

        step_input = 1

        continuous_tf = K/(J*s + B)

        discrete_tf = ct.sample_system(continuous_tf, sample_time, 'zoh' )
        difference_equation = DifferenceEquation(discrete_tf.num[0][0], discrete_tf.den[0][0])

        difference_equation_output = []

        difference_equation.tick(step_input)

        num_points = int(end_time/sample_time)+1

        T = np.linspace(0, 5, num_points)
        T, yout = ct.step_response(discrete_tf, T)

        for i in range(0, num_points-1):
            difference_equation.tick(1)

        for i in range(0, len(T)):
            self.assertAlmostEqual(yout[i], difference_equation.previous_output[i], 6)


if __name__ == '__main__':
    unittest.main()
