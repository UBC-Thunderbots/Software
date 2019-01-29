#include "geom/rectangle.h"

#include <algorithm>
#include <cmath>

#include "geom/util.h"

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
