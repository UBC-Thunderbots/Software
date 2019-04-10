//
// Created by roark on 28/03/19.
//

#include <algorithm>

namespace Util
{
    double linear(double value, double offset, double linear_width)
    {
        const double width_coef = 1 / linear_width;
        return std::clamp((width_coef * (value - offset) + 0.5), 0.0, 1.0);
    }

    unsigned int unitStep(double x, double a)
    {
        // Assuming u(0) = 1
        return x >= a ? 1 : 0;
    }

    unsigned int binary(double x, double a)
    {
        return unitStep(x, a);
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) / (in_max - in_min) * (out_max - out_min) + out_min;
    }

    double mapClamped(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return std::clamp(map(x, in_min, in_max, out_min, out_max), 0.0, 1.0);
    }
}  // namespace Util
