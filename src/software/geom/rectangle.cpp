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
    Vector add(amount, amount);
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
    std::vector<Point> corners = this->corners();

    return *std::max_element(corners.begin(), corners.end(),
                             [&](const Point& corner1, const Point& corner2) {
                                 return dist(corner1, p) < dist(corner2, p);
                             });
}

std::vector<Point> Rectangle::corners()
{
    return std::vector<Point>{operator[](0), operator[](1), operator[](2), operator[](3)};
}
