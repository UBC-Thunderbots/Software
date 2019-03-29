//
// Created by roark on 28/03/19.
//

#include

namespace Util
{
    double linear(double value, double offset, double linear_width)
    {
        double width_coef = 1/linear_width;
        return std::clamp ((width_coef*(value-offset) + 0.5), 0, 1);
    }
}
