#include "software/math/math_functions.h"

#include <algorithm>

double linear(double value, double offset, double linear_width)
{
    double width_coef = 1 / linear_width;
    return std::clamp((width_coef * (value - offset) + 0.5), 0.0, 1.0);
}

double rectangleSigmoid(const Rectangle& rect, const Point& point,
                        const double& sig_width)
{
    double x_offset = rect.centre().x();
    double y_offset = rect.centre().y();
    double x_size   = rect.xLength() / 2;
    double y_size   = rect.yLength() / 2;
    double x        = point.x();
    double y        = point.y();

    // For both x and y here we use two sigmoid functions centered at the positive and
    // negative edge of the rectangle respectively

    double x_val = std::min(sigmoid(x, x_offset + x_size, -sig_width),
                            sigmoid(x, x_offset - x_size, sig_width));

    double y_val = std::min(sigmoid(y, y_offset + y_size, -sig_width),
                            sigmoid(y, y_offset - y_size, sig_width));

    return x_val * y_val;
}

double circleSigmoid(const Circle& circle, const Point& point, const double& sig_width)
{
    // Calculate how far the point is from the circle center
    double distance_from_circle_center = (point - circle.origin()).length();

    return sigmoid(distance_from_circle_center, circle.radius(), -sig_width);
}

double sigmoid(const double& v, const double& offset, const double& sig_width)
{
    // This is factor that changes how quickly the sigmoid goes from 0 to 1 it. We
    // divide 8 by it because that is the distance a sigmoid function centered about 0
    // takes to go from 0.018 to 0.982 (and that is what the `sig_width` is, as per
    // the javadoc comment for this function)
    double sig_change_factor = 8 / sig_width;

    // TODO (#1985) there are much faster sigmoid implementations we can use
    // even max(tanh((2/sig_width) * (offset-v))) because tanh is faster than exp

    return 1 / (1 + std::exp(sig_change_factor * (offset - v)));
}

double percent_difference(double a, double b)
{
    if (a == 0 && b == 0) 
    {
        return 0;
    }
    return std::abs(a - b) / ((std::abs(a) + std::abs(b)) / 2);
}
