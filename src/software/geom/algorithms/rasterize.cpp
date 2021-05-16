#include <algorithm>
#include "software/geom/algorithms/rasterize.h"

#include "software/geom/algorithms/contains.h"


std::vector<Point> rasterize(const Circle &circle, const double resolution_size)
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

std::vector<Point> rasterize(const Rectangle &rectangle, const double resolution_size)
{
    std::vector<Point> points_covered;

    // Added resolution_size to the max to ensure that when Points are rasterized, all
    // Coordinates covering the entire image (including pixels that are not fully contained
    // by rectangle.
    for (double x = rectangle.xMin(); x < rectangle.xMax() + resolution_size; x += resolution_size)
    {
        for (double y = rectangle.yMin(); x < rectangle.yMax() + resolution_size; x += resolution_size)
        {
            points_covered.emplace_back(Point(x, y));
        }
    }
    return points_covered;
}

std::vector<Point> rasterize(const Polygon &polygon, const double resolution_size)
{
    // Using even-odd rule algorithm to fill in polygon
    // https://stackoverflow.com/a/31768384

    std::vector<Point> contained_points;
    std::vector<double> node_x;
    const auto& points = polygon.getPoints();

    auto max_point_y = [](const Point& a, const Point& b) {
        return a.y() < b.y();
    };
    auto max_point_x = [](const Point& a, const Point& b) {
        return a.x() < b.x();
    };

    // Calculate the highest and lowest x and y points
    double max_y = std::max_element(points.begin(), points.end(), max_point_y)->y();
    double min_y  = std::min_element(points.begin(), points.end(), max_point_y)->y();
    double max_x = std::max_element(points.begin(), points.end(), max_point_x)->x();
    double min_x  = std::min_element(points.begin(), points.end(), max_point_x)->x();

    int num_edges = static_cast<int>(points.size());

    // Loop through the rows of the image (i.e. polygon)
    for (double pixel_y = min_y; pixel_y < max_y; pixel_y += resolution_size) {
        int nodes = 0;
        int j = num_edges - 1;
        for (int i = 0; i < num_edges; i++) {
            if ((points[i].y() < pixel_y && points[j].y() >= pixel_y) ||
                (points[j].y() < pixel_y && points[i].y() >= pixel_y)) {
                // TODO Changed cast
                node_x[nodes++] = (points[i].x() +
                                         ((double) (pixel_y - points[i].y()) / (points[j].y() - points[i].y())) *
                                         (points[j].x() - points[i].x()));
            }
            j = i;
        }

        // TODO double check that the correct array is being sorted.
        std::sort(node_x.begin(), node_x.end());

        // Fill the pixels between node pairs
        for (int i = 0; i < nodes; i += 2) {
            if (node_x[i] >= max_x) break;
            if (node_x[i + 1] > min_x) {
                if (node_x[i] < min_x) node_x[i] = min_x;
                if (node_x[i + 1] > max_x) node_x[i + 1] = max_x;
                // TODO Casted to int?
                for (j = (int) node_x[i]; j < node_x[i + 1]; j++) {
                    contained_points.emplace_back(Point(j - min_x, pixel_y - min_y));
                }
            }

        }
    }

    return contained_points;
}
