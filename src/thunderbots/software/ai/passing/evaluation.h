/**
 * Declaration of evaluation functions for passing
 */

// TODO: if this file (or the corresponding `.cpp`) gets too big, they may need to be broken up
#pragma once

#include <functional>

#include "ai/world/field.h"
#include "geom/point.h"

namespace AI::Passing {

    /**
     * Calculates the static position quality for a given position on a given field
     *
     * @param field The field on which to calculate the static position quality
     * @param position The position on the field at which to calculate the quality
     *
     * @return A value in [0,1] representing the quality of the given point on the given field
     */
    double getStaticPositionQuality(Field field, Point position);

    /**
     * Calculates the value at the give point over a 2D sigmoid around the given rectangle
     *
     * The sigmoid constructed will be 0 far enough outside the rectangle, and 1
     * far enough within the rectangle
     *
     * @param rect The rectangle over which to make sigmoid function. The width of the
     *             the rectangle is considered to be in x, and the height in y
     * @param sig_width The length (in either x or y) required to cause the value of the
     *                 sigmoid to go from 0.5 to 0.982
     * @return The value of the sigmoid over the rectangle at the given point
     */
    double rectangleSigmoid(Rectangle rect, Point point, double sig_width);

    /**
     * A sigmoid function with a given offset from 0 and rate of change
     *
     * To flip the sigmoid around, use a negative sig_width
     *
     * @param v The value to evaluate over the sigmoid
     * @param offset The offset of the center of the  sigmoid from 0
     * @param sig_width The length (in either x or y) required to cause the value of the
     *                 sigmoid to go from 0.5 to 0.982
     * @return A value in [0,1] that is the value of the sigmoid at the value v
     */
    double sigmoid(double v, double offset, double sig_width);

}
