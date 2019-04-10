//
// Created by roark on 28/03/19.
//

#include <algorithm>

namespace Util
{
    double linear(double value, double offset, double linear_width)
    {
        double width_coef = 1 / linear_width;
        return std::clamp((width_coef * (value - offset) + 0.5), 0.0, 1.0);
    }

    unsigned int unitStep(double x, double a)
    {
        // Assuming u(0) = 1
        return x >= a ? 1 : 0;
    }
}  // namespace Util
