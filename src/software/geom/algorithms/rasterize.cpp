#include "software/geom/algorithms/rasterize.h"

#include "software/geom/algorithms/contains.h"


std::vector<Point> rasterize(const Circle &circle, double resolution_size)
{
//    std::set<Point> points_covered;
//
//    int radius              = circle.radius();
//    Point center_point      = circle.origin();
//    double xc = center_point.x();
//    double yc = center_point.y();
//
//    int x = 0, y = radius;
//    int d = 3 - 2 * radius;
//
//    auto set_blocked_coordinates = [points_covered, xc, yc](double x, double y){
//        points_covered.insert(Point(xc+x, yc+y));
//        points_covered.insert(Point(xc-x, yc+y));
//        points_covered.insert(Point(xc+x, yc-y));
//        points_covered.insert(Point(xc-x, yc-y));
//        points_covered.insert(Point(xc+y, yc+x));
//        points_covered.insert(Point(xc-y, yc+x));
//        points_covered.insert(Point(xc+y, yc-x));
//        points_covered.insert(Point(xc-y, yc-x));
//    };
//
//    set_blocked_coordinates(x, y);
//
//    while (y >= x)
//    {
//        x++;
//
//        if (d > 0)
//        {
//            y--;
//            d = d + 4 * (x - y) + 10;
//        }
//        else
//        {
//            d = d + 4 * x + 6;
//        }
//        set_blocked_coordinates(x, y);
//    }

    return std::vector<Point>();
}

std::vector<Point> rasterize(const Rectangle &rectangle, double resolution_size)
{
    std::vector<Point> points_covered;

    for (double x = rectangle.xMin(); x <= rectangle.xMax(); x += resolution_size)
    {
        for (double y = rectangle.yMin(); x <= rectangle.yMax(); x += resolution_size)
        {
            points_covered.emplace_back(Point(x, y));
        }
    }

    return points_covered;
}

std::vector<Point> rasterize(const Polygon &polygon, double resolution_size)
{
    return std::vector<Point>();
}