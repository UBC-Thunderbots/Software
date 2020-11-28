#ifndef UTIL_H
#define UTIL_H

/**
 * Finds the maximum value in the given array.
 *
 * @param  array [in] The array to parse through
 * @param  size  The size of the array
 *
 * @return       The max value in the array
 */
float fmax_of_array(float array[], unsigned size);

/**
 * Finds the minimum value in the given array.
 *
 * @param  array [in] The array to parse through
 * @param  size  The size of the array
 *
 * @return       The min value in the array
 */
float fmin_of_array(float array[], unsigned size);

/**
 * Find the index that corresponds to the max value in the array
 * @param  array [in] The array to parse through
 * @param  size  The size of the array
 *
 * @return       The index of the max value in the array
 */
unsigned argmax(float array[], unsigned size);

/**
 * Find the index that corresponds to the min value in the array
 * @param  array [in] The array to parse through
 * @param  size  The size of the array
 *
 * @return       The index of the min value in the array
 */
unsigned argmin(float array[], unsigned size);

/**
 * Get an array that has the absolute value of each item in the
 * given array
 * @param  array [in] The array to parse through
 * @param  abs_array [out] The array to parse through
 * @param  size  The size of the array
 */
void fabs_of_array(float array[], float abs_array[], unsigned size);

/**
 * Limits the value to be between
 * -limiting_value <= value <= limiting_value
 * @param value          The value to limit
 * @param limiting_value The limit to check against
 */
void limit(float *value, float limiting_value);

/**
 * Clamps the value to be between
 * lower <= value <= upper
 * @param value  The value to clamp
 * @param lower  The lower limit to check against
 * @param upper  The upper limit to check against
 */
void clamp(float *value, float lower, float upper);

/**
 * Converts a degrees value to radians
 *
 * @param  degrees an angle in degrees
 *
 * @return         an angle in radians
 */
float radians(float degrees);
#endif
