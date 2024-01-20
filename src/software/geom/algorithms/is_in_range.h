#pragma once

/**
 * Returns whether a value is within a range, inclusive of both ends. Note that the range
 * can be specified in either order.
 *
 * @param val the value to check
 * @param range1 the first value of the range
 * @param range2 the second value of the range
 * @return whether the value is within the range, inclusive of both ends
 */
bool isInRangeInclusive(double val, double range1, double range2);

/**
 * Returns whether a value is within a range, exclusive of either ends. Note that the
 * range can be specified in either order.
 *
 * @param val the value to check
 * @param range1 the first value of the range
 * @param range2 the second value of the range
 * @return whether the value is within the range, exclusive of either ends
 */
bool isInRangeExclusive(double val, double range1, double range2);
