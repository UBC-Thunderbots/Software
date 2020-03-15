#pragma once

/**
 * Returns true if two doubles are almost equal, false otherwise.
 * Almost equal is defined as the two doubles having a difference
 * of less than fixedEpsilon, or having a ULPs distance less than
 * ulpsEpsilon.
 *
 * Further reading: https://bitbashing.io/comparing-floats.html
 *
 * @param a first double to compare
 * @param b second double to compare
 * @param fixedEpsilon the fixed epsilon value for near-zero comparisons
 * @param ulpsEpsilon the maximum ULPs distance to consider two numbers almost equal
 * @return true if two doubles are almost equal, false otherwis
 */
bool almostEqual(double a, double b, double fixedEpsilon, int ulpsEpsilon);
