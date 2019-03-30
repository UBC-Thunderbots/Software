//
// Created by roark on 28/03/19.
//

#pragma once

namespace Util
{
    /**
     * Linearly maps an input value to an output value in the range [0,1]
     *
     * @param value The input value
     * @param offset The value by which to offset the input value
     * @param linear_width the total length required for the output value to go from 0 to
     * 1
     */
    double linear(double value, double offset, double linear_width);
}  // namespace Util
