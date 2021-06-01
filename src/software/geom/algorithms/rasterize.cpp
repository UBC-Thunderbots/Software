#include <algorithm>
#include <cmath>
#include "software/geom/algorithms/rasterize.h"

#include "software/geom/algorithms/contains.h"

// TODO When rasterizing without knowing the relative positions of the pixels, you may be off by 1 pixel in each
// axis. eg. A 1.5 x 1 rectangle may overlap with 2 or 3 pixels (assuming pixel dimension 1) depending on how it the rectangle
// is positioned.
std::vector<Point> rasterize(const Circle &circle, const double resolution_size)
{
//    std::vector<Point> covered_points;
//
//    Point origin = circle.origin();
//    double radius = circle.radius();
//
//    // Added resolution_size to the max to ensure that when Points are rasterized, all
//    // Coordinates (including pixels that are not fully contained by rectangle) are
//    // taken into account.
//    for (double x_offset = -radius; x_offset < radius + resolution_size; x_offset += resolution_size)
//    {
//        for (double y_offset = -radius; y_offset < radius + resolution_size; y_offset += resolution_size)
//        {
//            if(x_offset * x_offset + y_offset * y_offset < radius * radius)
//            {
//                covered_points.emplace_back(Point(origin.x() + x_offset, origin.y() + y_offset));
//            }
//        }
//    }

    std::vector<Point> covered_points;

    double diameter = circle.radius() * 2;
    Point origin = circle.origin();
    // max number of pixels in each dimension
    int max_num_pixels = (int) std::ceil(diameter / resolution_size);

    for (int x_pixel = 0; x_pixel <= max_num_pixels; x_pixel++)
    {
        // x and y offset from the top left corner of the rectangle
        double x_offset;

        // Adjust the last x and y pixels to be on the edge of the rectangle to make sure
        // that the points cover the entire rectangle without going outside.
        if (x_pixel == max_num_pixels)
        {
            x_offset = diameter;
        }
        else
        {
            x_offset = x_pixel * resolution_size;
        }

        double x_point = origin.x() - circle.radius() + x_offset;

        for (int y_pixel = 0; y_pixel <= max_num_pixels; y_pixel++)
        {
            double y_offset;
            if (y_pixel == max_num_pixels)
            {
                y_offset = diameter;
            }
            else
            {
                y_offset = y_pixel * resolution_size;
            }
            double y_point = origin.y() - circle.radius() + y_offset;

            // if point is inside circle
            if((x_point - origin.x()) * (x_point - origin.x()) + (y_point - origin.y()) * (y_point - origin.y()) <= circle.radius() * circle.radius())
            {
                covered_points.emplace_back(Point(x_point, y_point));
            }
        }
    }

    for (auto p = covered_points.begin(); p != covered_points.end(); ++p) // TODO Remove, added for testing
        std::cout << *p << ", ";
    std::cout << std::endl;

    return covered_points;
}

std::vector<Point> rasterize(const Rectangle &rectangle, const double resolution_size)
{
    std::vector<Point> covered_points;

    int num_pixels_x = (int) std::ceil(rectangle.xLength() / resolution_size);
    int num_pixels_y = (int) std::ceil(rectangle.yLength() / resolution_size);

    for (int x_pixel = 0; x_pixel <= num_pixels_x; x_pixel++)
    {
        // x and y offset from the top left corner of the rectangle
        double x_offset;

        // Adjust the last x and y pixels to be on the edge of the rectangle to make sure
        // that the points cover the entire rectangle without going outside.
        if (x_pixel == num_pixels_x)
        {
            x_offset = rectangle.xLength();
        }
        else
        {
            x_offset = x_pixel * resolution_size;
        }

        for (int y_pixel = 0; y_pixel <= num_pixels_y; y_pixel++)
        {
            double y_offset;
            if (y_pixel == num_pixels_y)
            {
                y_offset = rectangle.yLength();
            }
            else
            {
                y_offset = y_pixel * resolution_size;
            }

            covered_points.emplace_back(Point(rectangle.xMin() + x_offset, rectangle.yMin() + y_offset));
        }
    }

    for (auto p = covered_points.begin(); p != covered_points.end(); ++p) // TODO Remove, added for testing
        std::cout << *p << ", ";
    std::cout << std::endl;

    return covered_points;
}

std::vector<Point> rasterize(const Polygon &polygon, const double resolution_size)
{
    // Using even-odd rule algorithm to fill in polygon
    // https://stackoverflow.com/a/31768384
    std::vector<Point> contained_points;
    return contained_points;
}
