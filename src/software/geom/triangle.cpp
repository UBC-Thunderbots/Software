#include "software/geom/triangle.h"

#include "software/geom/point.h"

Triangle::Triangle(const Point &point1, const Point &point2, const Point &point3)
    : ConvexPolygon({point1, point2, point3})
{
}

Point Triangle::mean() const
{
    Point p1 = points_[0];
    Point p2 = points_[1];
    Point p3 = points_[2];

    double mean_x = (p1.x() + p2.x() + p3.x()) / 3;
    double mean_y = (p1.y() + p2.y() + p3.y()) / 3;

    Point mean = Point(mean_x, mean_y);

    return mean;
}
