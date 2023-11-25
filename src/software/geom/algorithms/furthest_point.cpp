#include "software/geom/algorithms/furthest_point.h"

#include <algorithm>

#include "software/geom/algorithms/distance.h"

Point furthestPoint(const Rectangle &a, const Point &b)
{
    std::vector<Point> corners = a.getPoints();

    return *std::max_element(corners.begin(), corners.end(),
                             [&](const Point &corner1, const Point &corner2) {
                                 return distance(b, corner1) < distance(b, corner2);
                             });
}
