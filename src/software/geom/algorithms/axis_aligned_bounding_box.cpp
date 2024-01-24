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

    double start_left = stadium.segment().getStart().x() - stadium.radius();
    double start_right = stadium.segment().getStart().x() + stadium.radius();
    double end_left = stadium.segment().getEnd().x() - stadium.radius();
    double end_right = stadium.segment().getEnd().x() + stadium.radius();
    double start_bottom = stadium.segment().getStart().y() - stadium.radius();
    double start_top = stadium.segment().getStart().y() + stadium.radius();
    double end_bottom = stadium.segment().getEnd().y() - stadium.radius();
    double end_top = stadium.segment().getEnd().y() + stadium.radius();

    double min_x = std::min(end_right, std::min(end_left, std::min(start_left, start_right)));
    double max_x = std::max(end_right, std::max(end_left, std::max(start_left, start_right)));

    double min_y = std::min(end_top, std::min(end_bottom, std::min(start_bottom, start_top)));
    double max_y = std::max(end_top, std::max(end_bottom, std::max(start_bottom, start_top)));

    Point bottom_left(min_x - inflation_radius, min_y - inflation_radius);
    Point top_right(max_x + inflation_radius, max_y + inflation_radius);
    return Rectangle(bottom_left, top_right);
}
