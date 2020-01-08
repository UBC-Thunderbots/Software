#include "software/new_geom/point.h"
#include "software/new_geom/triangle.h"

Triangle::Triangle(const Point &point1, const Point &point2, const Point &point3) : ConvexPolygon({point1, point2, point3}) {}

Point Triangle::center() const
{
    Point p1 = points_[0];
    Point p2 = points_[1];
    Point p3 = points_[2];

    double center_x = (p1.x() + p2.x() + p3.x()) / 3;
    double center_y = (p1.y() + p2.y() + p3.y()) / 3;

    Point center = Point(center_x, center_y);

    return center;
}
