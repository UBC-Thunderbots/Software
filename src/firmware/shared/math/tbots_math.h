#pragma once
/**
 * This file is not named `math.h` to avoid naming conflicts with the standard `math.h`
 */

#include <math.h>

// define our own PI value here that is a float because M_PI in math.h is a double
#define P_PI 3.14159265f

/**
 * Function performs a linear interpolation between the points [x0 ,y0] and [x1, y1] at
 * the position xp
 *
 * @pre x1 < xp < x2, or in other words - xp must be between x0 and x1 AND x0 and x1
 * CANNOT be equal
 *
 * @param x0 The lower-bound x value
 *
 * @param y0 The lower-bound y value
 *
 * @param x1 The upper-bound x value
 *
 * @param y1 The upper-bound y value
 *
 * @param xp The intermediate x value. The linear interpolation is performed to get the
 * estimated value of y at this point
 *
 * @return The estimated value of the function at xp
 */
float shared_tbots_math_linearInterpolation(float x0, float y0, float x1, float y1,
                                            float xp);
