#include "software/new_geom/util/distance.h"

#define POINT_BOOST_COMPATABILITY_THIS_IS_NOT_IN_A_HEADER
#include "software/new_geom/point_boost_geometry_compatability.h"

double distance(const Line &first, const Point &second)
{
    Line::Coeffs coeffs = first.getCoeffs();
    return abs(coeffs.a * second.x() + coeffs.b * second.y() + coeffs.c) /
           hypot(coeffs.a, coeffs.b);
}

double distance(const Point &first, const Line &second)
{
    return distance(second, first);
}

double distance(const Point &first, const Point &second)
{
    return (first - second).length();
}

double distance(const Segment &first, const Segment &second)
{
    boost::geometry::model::segment<Point> AB(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> CD(second.getSegStart(),
                                              second.getEnd());  // similar code
    bool intersects = boost::geometry::intersects(AB, CD);
    if (intersects)
    {
        return 0.0;
    }
    double first_to_second_start_distsq = distanceSquared(first, second.getSegStart());
    double first_to_second_end_distsq   = distanceSquared(first, second.getEnd());
    double second_to_first_start_distsq = distanceSquared(second, first.getSegStart());
    double second_to_first_end_distsq   = distanceSquared(second, first.getEnd());
    return std::sqrt(
        std::min({first_to_second_start_distsq, first_to_second_end_distsq,
                  second_to_first_start_distsq, second_to_first_end_distsq}));
}

double distance(const Point &first, const Segment &second)
{
    return std::sqrt(distanceSquared(first, second));
}

double distance(const Segment &first, const Point &second)
{
    return distance(second, first);
}

double distance(const Point &first, const Polygon &second)
{
    if (second.contains(first))
    {
        return 0;
    }

    double min_dist = std::numeric_limits<double>::max();

    // Calculate the distance from the point to each edge
    for (auto &segment : second.getSegments())
    {
        double current_dist = distance(first, segment);
        if (current_dist < min_dist)
        {
            min_dist = current_dist;
        }
    }
    return min_dist;
}

double distance(const Polygon &first, const Point &second)
{
    return distance(second, first);
}

double distance(const Point &first, const Circle &second)
{
    double distance_from_edge =
        (distance(first, second.getOrigin()) - second.getRadius());
    return distance_from_edge > 0 ? distance_from_edge : 0;
}

double distance(const Circle &first, const Point &second)
{
    return distance(second, first);
}

double distanceSquared(const Point &first, const Segment &second)
{
    Vector seg_vec            = second.toVector();
    Vector seg_start_to_point = first - second.getSegStart();

    if (seg_vec.dot(seg_start_to_point) <= 0)
    {
        return seg_start_to_point.lengthSquared();
    }

    // We delay this calculation until we actually need it for performance reasons
    Vector seg_end_to_point = first - second.getEnd();

    if (seg_vec.dot(seg_end_to_point) >= 0)
    {
        return seg_end_to_point.lengthSquared();
    }

    double cross = seg_start_to_point.cross(seg_vec);
    return std::fabs(cross * cross / seg_vec.lengthSquared());
}

double distanceSquared(const Segment &first, const Point &second)
{
    return distanceSquared(second, first);
}

double distanceSquared(const Point &first, const Point &second)
{
    return (first - second).lengthSquared();
}
