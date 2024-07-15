#include "software/geom/algorithms/distance.h"

#include "software/geom/algorithms/contains.h"

double distance(const Line &first, const Point &second)
{
    Line::Coeffs coeffs = first.getCoeffs();
    return std::abs(coeffs.a * second.x() + coeffs.b * second.y() + coeffs.c) /
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
    if (contains(second, first))
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
    double distance_from_edge = (distance(first, second.origin()) - second.radius());
    return distance_from_edge > 0 ? distance_from_edge : 0;
}

double distance(const Circle &first, const Point &second)
{
    return distance(second, first);
}

double distance(const Point &first, const Stadium &second)
{
    return std::max(distance(first, second.segment()) - second.radius(), 0.0);
}

double distance(const Stadium &first, const Point &second)
{
    return distance(second, first);
}

double distanceSquared(const Point &first, const Segment &second)
{
    Vector seg_vec            = second.toVector();
    Vector seg_start_to_point = first - second.getStart();

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
