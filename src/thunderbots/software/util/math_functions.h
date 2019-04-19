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
     * @param offset The input value at which the linear function will return 0.5
     * @param linear_width the total length required for the output value to go from 0 to
     * 1
     */
    double linear(double value, double offset, double linear_width);

    /**
     * Unit step function with output {0, 1}
     *
     * @param[in] double x: The input variable
     * @param[in] double a: The offset constant
     * 
     * @return The evaluation of the math function u(x + a)
     */
    unsigned int unitStep(double x, double a);

    /**
     * ??? not sure if needed
     * Binary function that basically mirrors unit step function
     * except it uses a different name
     * 
     * @param[in] double x: The input variable
     * @param[in] double a: The offset constant
     * 
     * @return The result of unitStep(x, a)
     */
    unsigned int binary(double x, double a);

    /**
     * Map values from one range to another range
     * (A more generalized function of `linear`)
     * 
     * @param[in] double x: The input variable
     * @param[in] double in_min: The input range minimum
     * @param[in] double in_max: The input range maximum
     * @param[in] double out_min: The output range minimum
     * @param[in] double out_max: The output range maximum
     * 
     * @return The result mapped number
     */
    double map(double x, double in_min, double in_max, double out_min, double out_max);

    /**
     * Constrained map function -- same as map function
     * except the output is clamped to [0, 1]
     * 
     * @param[in] double x: The input variable
     * @param[in] double in_min: The input range minimum
     * @param[in] double in_max: The input range maximum
     * @param[in] double out_min: The output range minimum
     * @param[in] double out_max: The output range maximum
     *
     * @return The result mapped number clampped between [0, 1]
     */
    double mapClamped(double x, double in_min, double in_max, double out_min, double out_max);
}  // namespace Util
