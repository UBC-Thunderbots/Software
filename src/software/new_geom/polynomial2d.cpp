#include "software/new_geom/polynomial2d.h"

Polynomial2d::Polynomial2d() : poly_x(), poly_y() {}

Polynomial2d::Polynomial2d(Polynomial1d poly_x, Polynomial1d poly_y)
    : poly_x(poly_x), poly_y(poly_y)
{
}

Polynomial2d::Polynomial2d(std::vector<Point> points)
{
    if (points.size() < 2)
    {
        throw std::invalid_argument(
            "Given less then two points to construct a 2d Polynomial, which has no unique solution.");
    }

    // Setup x polynomial
    {
        std::vector<std::pair<double, double>> constraints;
        for (size_t i = 0; i < points.size(); i++)
        {
            double t = static_cast<double>(i) / static_cast<double>(points.size()-1);
            constraints.emplace_back(
                std::make_pair<double, double>(std::move(t), points[i].x()));
        }
        poly_x = Polynomial1d(constraints);
    }

    // Setup y polynomial
    {
        std::vector<std::pair<double, double>> constraints;
        for (size_t i = 0; i < points.size(); i++)
        {
            double t = static_cast<double>(i) / static_cast<double>(points.size()-1);
            constraints.emplace_back(
                std::make_pair<double, double>(std::move(t), points[i].y()));
        }
        poly_y = Polynomial1d(constraints);
    }
}

Point Polynomial2d::valueAt(double val) const
{
    return Point(poly_x.valueAt(val), poly_y.valueAt(val));
}

Polynomial1d Polynomial2d::getPolyX() const
{
    return poly_x;
}

Polynomial1d Polynomial2d::getPolyY() const
{
    return poly_y;
}
