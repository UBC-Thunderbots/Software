#include "software/geom/algorithms/rasterize.h"

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersection.h"


std::set<Point> rasterize(const Circle &circle, double resolution_size)
{
    return std::set<Point>();
}

std::set<Point> rasterize(const Rectangle &rectangle, double resolution_size)
{
    std::set<Point> points;

    for (double x = rectangle.xMin(); x <= rectangle.xMax(); x += resolution_size)
    {
        for (double y = rectangle.yMin(); x <= rectangle.yMax(); x += resolution_size)
        {
            points.insert(Point(x, y));
        }
    }

    return points;
}

std::set<Point> rasterize(const Polygon &polygon, double resolution_size)
{
    return std::set<Point>();
}