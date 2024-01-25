#include "software/geom/algorithms/axis_aligned_bounding_box.h"

Rectangle axisAlignedBoundingBox(const Circle& circle, const double inflation_radius)
{
    Point bottom_left(circle.origin().x() - circle.radius() - inflation_radius,
                      circle.origin().y() - circle.radius() - inflation_radius);
    Point top_right(circle.origin().x() + circle.radius() + inflation_radius,
                    circle.origin().y() + circle.radius() + inflation_radius);
    return Rectangle(bottom_left, top_right);
}

Rectangle axisAlignedBoundingBox(const Rectangle& rectangle,
                                 const double inflation_radius)
{
    if (inflation_radius == 0)
    {
        return rectangle;
    }

    return rectangle.expand(inflation_radius);
}

Rectangle axisAlignedBoundingBox(const Polygon& polygon, const double inflation_radius)
{
    double min_x = std::numeric_limits<double>::max();
    double min_y = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double max_y = std::numeric_limits<double>::lowest();
    for (const Point& point : polygon.getPoints())
    {
        min_x = std::min(min_x, point.x());
        min_y = std::min(min_y, point.y());
        max_x = std::max(max_x, point.x());
        max_y = std::max(max_y, point.y());
    }
    Point bottom_left(min_x - inflation_radius, min_y - inflation_radius);
    Point top_right(max_x + inflation_radius, max_y + inflation_radius);
    return Rectangle(bottom_left, top_right);
}

Rectangle axisAlignedBoundingBox(const Stadium& stadium,
                                 const double inflation_radius)
{
    double min_x = std::min(stadium.segment().getEnd().x(),
                            stadium.segment().getStart().x()) - stadium.radius();
    double max_x = std::max(stadium.segment().getEnd().x(),
                            stadium.segment().getStart().x()) + stadium.radius();

    double min_y = std::min(stadium.segment().getEnd().y(),
                            stadium.segment().getStart().y()) - stadium.radius();
    double max_y = std::max(stadium.segment().getEnd().y(),
                            stadium.segment().getStart().y()) + stadium.radius();

    Point bottom_left(min_x - inflation_radius, min_y - inflation_radius);
    Point top_right(max_x + inflation_radius, max_y + inflation_radius);
    return Rectangle(bottom_left, top_right);
}
