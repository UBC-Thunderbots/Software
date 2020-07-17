#pragma once

/**
 * Returns true if two doubles are almost equal, false otherwise.
 * Almost equal is defined as the two doubles having a difference
 * of less than fixed_epsilon, or having a ULPs distance less than
 * ulps_epsilon.
 *
 * Further reading: https://bitbashing.io/comparing-floats.html
 *
 * @param a first double to compare
 * @param b second double to compare
 * @param fixed_epsilon the fixed epsilon value for near-zero comparisons
 * @param ulps_epsilon the maximum ULPs distance to consider two numbers almost equal
 * @return true if two doubles are almost equal, false otherwise
 */
bool almostEqual(double a, double b, double fixed_epsilon, int ulps_epsilon);
