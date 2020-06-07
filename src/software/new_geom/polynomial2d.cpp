#include "software/new_geom/polynomial2d.h"

Polynomial2d::Polynomial2d() : poly_x(), poly_y() {}

Polynomial2d::Polynomial2d(Polynomial1d poly_x, Polynomial1d poly_y)
    : poly_x(poly_x), poly_y(poly_y)
{
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
