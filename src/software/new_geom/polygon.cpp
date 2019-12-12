#include "software/new_geom/polygon.h"

#include <unordered_set>

Polygon::Polygon(const std::vector<Point>& points)
        : segments_(points.size()), points_(points)
{
    for (unsigned i = 0; i < points.size(); i++)
    {
        // add a segment between consecutive points, but wrap index
        // to draw a segment from the last point to first point.
        segments_[i] = Segment{points[i], points[(i + 1) % points.size()]};
    }
}

Polygon::Polygon(const std::initializer_list<Point>& points)
        : segments_(points.size()), points_(points)
{
    for (unsigned i = 0; i < points_.size(); i++)
    {
        // add a segment between consecutive points, but wrap index
        // to draw a segment from the last point to first point.
        segments_[i] =
                Segment{*(points.begin() + i), *(points.begin() + ((i + 1) % points.size()))};
    }
}

bool Polygon::contains(const Point& p) const
{
    // This algorithm is from https://stackoverflow.com/a/16391873
    // but does not include the bounding boxes.
    //
    // A quick description of the algorithm (also from the same post) is as follows:
    // "I run a semi-infinite ray horizontally (increasing x, fixed y) out from the test
    // point, and count how many edges it crosses. At each crossing, the ray switches
    // between inside and outside. This is called the Jordan curve theorem."
    //
    // NOTE: This algorithm will treat boundaries on the bottom-left of the polygon
    // different from the boundaries on the top-right of the polygon. This small
    // inconsistency does not matter for our use cases, and actually has the benefit that
    // should two distinct polygons share an edge, any point along this edge will be
    // located in one and only one polygon.
    bool point_is_contained = false;
    unsigned i              = 0;
    unsigned j              = points_.size() - 1;
    while (i < points_.size())
    {
        if (((points_.at(i).y() > p.y()) != (points_.at(j).y() > p.y())) &&
            (p.x() < (points_.at(j).x() - points_.at(i).x()) *
                         (p.y() - points_.at(i).y()) /
                         (points_.at(j).y() - points_.at(i).y()) +
                             points_.at(i).x()))
        {
            point_is_contained = !point_is_contained;
        }

        j = i++;
    }

    return point_is_contained;
}

const std::vector<Segment>& Polygon::getSegments() const
{
    return segments_;
}

const std::vector<Point>& Polygon::getPoints() const
{
    return points_;
}
