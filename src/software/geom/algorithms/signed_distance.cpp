#include "software/geom/algorithms/signed_distance.h"
#include "software/geom/algorithms/distance.h"

#include <algorithm>

double signedDistance(const Circle &first, const Point &second)
{
    return distance(first.origin(), second) - first.radius();
}
double signedDistance(const Point &first, const Circle &second)
{
    return signedDistance(second, first);
}

double signedDistance(const Rectangle &first, const Point &second)
{
    Vector centre_to_point = second - first.centre();
    Vector abs_centre_to_point = Vector(std::abs(centre_to_point.x()), std::abs(centre_to_point.y()));
    Vector corner_to_point =  abs_centre_to_point - (first.posXPosYCorner() - first.centre());
    Vector clamped_corner_to_point = Vector(std::max(corner_to_point.x(), 0.0), std::max(corner_to_point.y(), 0.0));

    return clamped_corner_to_point.length() + std::min(std::max(corner_to_point.x(), corner_to_point.y()), 0.0);
}
double signedDistance(const Point &first, const Rectangle &second)
{
    return signedDistance(second, first);
}

double signedDistance(const Polygon &first, const Point &second)
{
    std::vector<Point> points = first.getPoints();

    double min_length = (second-points[0]).lengthSquared();
    double s = 1.0;
    for(auto it = points.begin(), j=--points.end(); it != points.end(); j=it, ++it)
    {
        Vector e = j->toVector() - it->toVector();
        Vector w = second.toVector() - it->toVector();
        Vector b = w - e*std::clamp(w.dot(e) / e.dot(e), 0.0, 1.0);
        min_length = std::min(min_length, b.lengthSquared());
        if((second.y() >= it->y() && second.y() < j->y() && e.x()*w.y()>e.y()*w.x()) ||
           (second.y() < it->y() && second.y() >= j->y() && e.x()*w.y()<=e.y()*w.x()))
        {
            s *= -1;
        }
    }
    return s*sqrt(min_length);
}
double signedDistance(const Point &first, const Polygon &second)
{
    return signedDistance(second, first);
}

double signedDistance(const Stadium &first, const Point &second)
{
    return distance(first.segment(), second) - first.radius();
}
double signedDistance(const Point &first, const Stadium &second)
{
    return signedDistance(second, first);
}