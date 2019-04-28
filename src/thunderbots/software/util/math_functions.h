/*
 * This file contains declarations for math functions
 */

#pragma once

#include "geom/circle.h"
#include "geom/point.h"
#include "geom/rectangle.h"

namespace Util
{
    /**
     * Linearly maps an input value to an output value in the range [0,1]
     *
     * @param value The input value
     * @param offset The input value at which the linear function will return 0.5
     * @param linear_width the total length required for the output value to go from 0 to
     * 1
     */
    double linear(double value, double offset, double linear_width);

}  // namespace Util

/**
 * Calculates the value at the given point over a 2D sigmoid over the given rectangle
 *
 * The sigmoid constructed will approach 0 far enough outside the rectangle, and
 * approach 1 far enough within the rectangle. The value on the edge of the rectangle
 * will be 0.5
 *
 * @param rect The rectangle over which to make sigmoid function. The width of the
 *             the rectangle is considered to be in x, and the height in y
 * @param sig_width The length (in either x or y) required to cause the value of the
 *                 sigmoid to go from 0.018 to 0.982
 *
 * @return A value in [0,1], representing the value of the 2D sigmoid function over
 *         the given rectangle at the given point
 */
double rectangleSigmoid(const Rectangle& rect, const Point& point,
                        const double& sig_width);

/**
 * Calculates the value at the given point over a 2D sigmoid over the given circle
 *
 * The sigmoid constructed will approach 0 far enough outside the circle, and approach
 * 1 far enough within the circle. The value on the edge of the circle will be 0.5
 *
 * @param circle The circle over which to make sigmoid function
 * @param sig_width The length required to cause the value of the sigmoid to go from
 *                  0.018 to 0.982 across the edge of the circle
 *
 * @return A value in [0,1], representing the value of the 2D sigmoid function over
 *         the given circle at the given point
 */
double circleSigmoid(const Circle& circle, const Point& point, const double& sig_width);

/**
 * A sigmoid function with a given offset from 0 and rate of change
 *
 * By default this increases from -v to positive v, ie. y = 1 / (1+e^(-x))
 * To flip the sigmoid around (ie. increasing from +v to -v), subtract it from 1
 *
 * When using this function, it is strongly encouraged that you look at it's
 * implementation and go plot it, play around with the numbers a bit to really
 * understand what they're doing.
 *
 * @param v The value to evaluate over the sigmoid
 * @param offset The offset of the center of the  sigmoid from 0
 * @param sig_width The length (in either x or y) required to cause the value of the
 *                 sigmoid to go from 0.018 to 0.982
 *
 * @return A value in [0,1] that is the value of the sigmoid at the value v
 */
double sigmoid(const double& v, const double& offset, const double& sig_width);
