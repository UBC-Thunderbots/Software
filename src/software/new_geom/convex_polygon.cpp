#include "software/new_geom/convex_polygon.h"

#include <string>

ConvexPolygon::ConvexPolygon(const std::vector<Point>& points) : Polygon(points)
{
    if (!isConvex())
    {
        throw std::invalid_argument("Points do not make a convex polygon");
    }
}

ConvexPolygon::ConvexPolygon(const std::initializer_list<Point>& points) : Polygon(points)
{
    if (!isConvex())
    {
        throw std::invalid_argument("Points do not make a convex polygon");
    }
}

bool ConvexPolygon::isConvex()
{
    // TODO: Properly implement this (Issue #1140)
    // Temporarily always returning true to avoid errors until Issue #1140 is completed
    return true;
}

double ConvexPolygon::area() const
{
    // Algorithm taken from http://mathwords.com/a/area_convex_polygon.htm
    //
    // A = (1/2) * [(x1*y2 + x2y3 + x3y4 + ... + xny1) - (y1x2 + y2x3 + y3x4 + ... +
    // ynx1)] Coordinates must be taken in counterclockwise order around the polygon,
    // beginning and ending in the same point.
    std::vector<Point> reversePoints = points_;
    std::reverse(reversePoints.begin(), reversePoints.end());

    double first_term  = 0;
    double second_term = 0;

    unsigned num_points = reversePoints.size();

    for (unsigned i = 0; i < num_points; i++)
    {
        first_term += reversePoints[i].x() * reversePoints[(i + 1) % num_points].y();
        second_term += reversePoints[i].y() * reversePoints[(i + 1) % num_points].x();
    }

    return std::abs(first_term - second_term) / 2;
}
