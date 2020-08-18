#include "software/geom/polynomial2d.h"

Polynomial2d::Polynomial2d() : poly_x(), poly_y() {}

Polynomial2d::Polynomial2d(Polynomial1d poly_x, Polynomial1d poly_y)
    : poly_x(poly_x), poly_y(poly_y)
{
}

Polynomial2d::Polynomial2d(const std::vector<Point> &points)
{
    if (points.size() < 2)
    {
        throw std::invalid_argument(
            "Given less then two points to construct a 2d Polynomial, which has no unique solution.");
    }

    // Setup x polynomial
    {
        std::vector<Polynomial1d::Constraint> constraints;
        for (size_t i = 0; i < points.size(); i++)
        {
            double t = static_cast<double>(i) / static_cast<double>(points.size() - 1);
            constraints.emplace_back(
                Polynomial1d::Constraint{.input = t, .output = points[i].x()});
        }
        poly_x = Polynomial1d(constraints);
    }

    // Setup y polynomial
    {
        std::vector<Polynomial1d::Constraint> constraints;
        for (size_t i = 0; i < points.size(); i++)
        {
            double t = static_cast<double>(i) / static_cast<double>(points.size() - 1);
            constraints.emplace_back(
                Polynomial1d::Constraint{.input = t, .output = points[i].y()});
        }
        poly_y = Polynomial1d(constraints);
    }
}

Polynomial2d::Polynomial2d(std::initializer_list<Point> points)
    : Polynomial2d(std::vector<Point>(points))
{
}

Point Polynomial2d::getValueAt(double val) const
{
    return Point(poly_x.valueAt(val), poly_y.valueAt(val));
}

const Polynomial1d &Polynomial2d::getPolyX() const
{
    return poly_x;
}

const Polynomial1d &Polynomial2d::getPolyY() const
{
    return poly_y;
}

Polynomial2d operator+(const Polynomial2d &p1, const Polynomial2d &p2)
{
    return Polynomial2d(p1.getPolyX() + p2.getPolyX(), p1.getPolyY() + p2.getPolyY());
}

Polynomial2d operator-(const Polynomial2d &p1, const Polynomial2d &p2)
{
    return Polynomial2d(p1.getPolyX() - p2.getPolyX(), p1.getPolyY() - p2.getPolyY());
}

Polynomial2d &operator+=(Polynomial2d &p1, const Polynomial2d &p2)
{
    return p1 = p1 + p2;
}

Polynomial2d &operator-=(Polynomial2d &p1, const Polynomial2d &p2)
{
    return p1 = p1 - p2;
}

bool operator==(const Polynomial2d &p1, const Polynomial2d &p2)
{
    return (p1.getPolyX() == p2.getPolyX()) && (p1.getPolyY() == p2.getPolyY());
}
