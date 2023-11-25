#include "software/geom/algorithms/calculate_block_cone.h"

Point calculateBlockCone(const Vector &a, const Vector &b, const double radius)
{
    if (a.length() < FIXED_EPSILON || b.length() < FIXED_EPSILON)
    {
    }
    // unit vector and bisector
    Vector au = a / a.length();
    Vector c  = au + b / b.length();
    // use similar triangle
    return Point(c * (radius / std::fabs(au.cross(c))));
}

Point calculateBlockCone(const Point &a, const Point &b, const Point &p,
                         const double radius)
{
    return p + (calculateBlockCone(a - p, b - p, radius)).toVector();
}
