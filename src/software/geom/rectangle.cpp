#include "software/geom/rectangle.h"

#include <algorithm>
#include <cmath>

#include "software/geom/util.h"

bool Rectangle::expand(double amount)
{
    if (diagonal.x() < -2 * amount || diagonal.y() < -2 * amount)
    {
        return false;
    }
    Point add(amount, amount);
    min_corner -= add;
    diagonal += 2 * add;
    return true;
}

double Rectangle::distToBoundary(Point p)
{
    double inf = 10e9;  // approx of infinity
    for (unsigned int i = 0; i < 4; i++)
    {
        Point a = operator[](i);
        Point b = operator[](i + 1);
        inf     = std::min(inf, dist(p, Segment(a, b)));
    }
    return inf;
}

Point Rectangle::furthestCorner(Point p)
{
    Point furthestPoint = operator[](0);
    double furthestDist = dist(p, operator[](0));

    for (int i = 1; i < 4; i++)
    {
        double newDist = dist(p, operator[](i));
        if (newDist > furthestDist)
        {
            furthestPoint = operator[](i);
            furthestDist  = newDist;
        }
    }
    return furthestPoint;
}
