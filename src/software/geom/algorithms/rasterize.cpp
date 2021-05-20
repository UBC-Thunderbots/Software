#include <algorithm>
#include "software/geom/algorithms/rasterize.h"

#include "software/geom/algorithms/contains.h"

// TODO When rasterizing without knowing the relative positions of the pixels, you may be off by 1 pixel in each
// axis. eg. A 1.5 x 1 rectangle may overlap with 2 or 3 pixels (assuming pixel dimension 1) depending on how it the rectangle
// is positioned.
std::vector<Point> rasterize(const Circle &circle, const double resolution_size)
{
    std::vector<Point> covered_points;

    Point origin = circle.origin();
    double radius = circle.radius();

    // Added resolution_size to the max to ensure that when Points are rasterized, all
    // Coordinates (including pixels that are not fully contained by rectangle) are
    // taken into account.
    for (double x_offset = -radius; x_offset < radius + resolution_size; x_offset += resolution_size)
    {
        for (double y_offset = -radius; y_offset < radius + resolution_size; y_offset += resolution_size)
        {
            if(x_offset * x_offset + y_offset * y_offset < radius * radius)
            {
                covered_points.emplace_back(Point(origin.x() + x_offset, origin.y() + y_offset));
            }
        }
    }
    return covered_points;

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
    std::vector<Point> covered_points;

    // Added resolution_size to the max to ensure that when Points are rasterized, all
    // Coordinates (including pixels that are not fully contained by rectangle) are
    // taken into account.
    for (double x = rectangle.xMin(); x < rectangle.xMax() + resolution_size; x += resolution_size)
    {
        for (double y = rectangle.yMin(); y < rectangle.yMax() + resolution_size; y += resolution_size)
        {
            covered_points.emplace_back(Point(x, y));
        }
    }
    return covered_points;
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

        // TODO double check that the array is being sorted in correct order.
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
