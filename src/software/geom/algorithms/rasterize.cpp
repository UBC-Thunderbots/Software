#include <algorithm>
#include <cmath>
#include "software/geom/algorithms/rasterize.h"

#include "software/geom/algorithms/contains.h"

// TODO When rasterizing without knowing the relative positions of the pixels, you may be off by 1 pixel in each
// axis. eg. A 1.5 x 1 rectangle may overlap with 2 or 3 pixels (assuming pixel dimension 1) depending on how it the rectangle
// is positioned.
std::vector<Point> rasterize(const Circle &circle, const double resolution_size)
{
    std::vector<Point> covered_points;

    // Using an approach to find the points on the edges using y = +-sqrt(r^2 - (x - k)^2) + h and filling in the rest
    // Downside is using sqrt to calculate
    // Chosen to avoid points being outside of circle due to floating point errors
    const double EPSILON = 0.0001;
    double radius = circle.radius() - EPSILON;
    double diameter = radius * 2;
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

        double x_point = origin.x() - radius + x_offset;
        double y_sqrt  = std::sqrt(std::abs(radius * radius - (x_point - origin.x()) * (x_point - origin.x())));
        double y_min   = -y_sqrt + origin.y();
        double y_max   = y_sqrt + origin.y();

        int y_num_pixels = (int) std::ceil((y_max - y_min) / resolution_size);
        for (int y_pixel = 0; y_pixel <= y_num_pixels; y_pixel++)
        {
            double y_offset;
            if (y_pixel == y_num_pixels)
            {
                y_offset = y_max - y_min;
            }
            else
            {
                y_offset = y_pixel * resolution_size;
            }
            double y_point = y_min + y_offset;

            covered_points.emplace_back(Point(x_point, y_point));
        }
    }

//    for (auto p = covered_points.begin(); p != covered_points.end(); ++p) // TODO Remove, added for testing
//        std::cout << *p << ", ";
//    std::cout << std::endl;

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

//    for (auto p = covered_points.begin(); p != covered_points.end(); ++p) // TODO Remove, added for testing
//        std::cout << *p << ", ";
//    std::cout << std::endl;

    return covered_points;
}

std::vector<Point> rasterize(const Polygon &polygon, const double resolution_size)
{
    // Using even-odd rule algorithm to fill in polygon
    // https://stackoverflow.com/a/31768384
    std::vector<Point> contained_points;
    return contained_points;
}
