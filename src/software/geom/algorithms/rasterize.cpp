#include "software/geom/algorithms/rasterize.h"

#include <algorithm>
#include <cmath>

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/geom_constants.h"

std::vector<Point> rasterize(const Circle& circle, const double resolution_size)
{
    // Iterating through the x values from left (origin - radius) to right (origin +
    // radius) of the circle which are resolution_size apart. Then finding the min and max
    // y values for each x value using:
    //             y = +-sqrt(r^2 - (x - k)^2) + h      (<== (y - h)^2 + (x - k)^2 = r^2)
    // and filling in the points resolution_size away from each other between them.
    std::vector<Point> covered_points;

    // Offset for the radius to avoid points being outside of the circle due to floating
    // point errors
    double radius   = circle.radius() - FIXED_EPSILON;
    double diameter = radius * 2;
    Point origin    = circle.origin();

    // max number of pixels in each dimension
    int max_num_pixels = static_cast<int>(std::ceil(diameter / resolution_size));

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
        double y_sqrt  = std::sqrt(
            std::abs(radius * radius - (x_point - origin.x()) * (x_point - origin.x())));
        double y_min = -y_sqrt + origin.y();
        double y_max = y_sqrt + origin.y();

        int y_num_pixels = static_cast<int>(std::ceil((y_max - y_min) / resolution_size));
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

    // Draw points resolution_size away (arclength) from each other over the circumference
    double delta_theta     = resolution_size / radius;
    int num_theta_sections = static_cast<int>(std::ceil(M_PI / delta_theta));
    for (int section = 0; section < num_theta_sections; section++)
    {
        double theta    = section * delta_theta;
        double x_offset = std::cos(theta) * radius;
        double y_offset = std::sin(theta) * radius;

        covered_points.emplace_back(Point(origin.x() + x_offset, origin.y() + y_offset));
        covered_points.emplace_back(Point(origin.x() + x_offset, origin.y() - y_offset));
    }

    return covered_points;
}

std::vector<Point> rasterize(const Rectangle& rectangle, const double resolution_size)
{
    // Loop through the length and the width of the rectangle and add points
    // resolution_size away from each other. Also making sure that the edges are also
    // filled with points if the dimensions of the rectangle are not divisible by
    // resolution_size.
    std::vector<Point> covered_points;

    int num_pixels_x = static_cast<int>(std::ceil(rectangle.xLength() / resolution_size));
    int num_pixels_y = static_cast<int>(std::ceil(rectangle.yLength() / resolution_size));

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

            covered_points.emplace_back(
                Point(rectangle.xMin() + x_offset, rectangle.yMin() + y_offset));
        }
    }

    return covered_points;
}

std::vector<Point> rasterize(const Polygon& polygon, const double resolution_size)
{
    // This rasterize function uses two steps to fully cover a polygon.
    // 1. Draw a bounding box around the polygon and loop through it using a similar
    // approach as rasterize for polygon and only add the points that are contained by the
    // polygon.
    //                                     AND
    // 2. To ensure that the edges are covered, draw a line between every two consecutive
    // vertices and draw points on the line, resolution_size or less away from each other.

    // Used to avoid points being outside of polygon due to floating point errors
    const auto& polygon_vertices = polygon.getPoints();
    std::vector<Point> covered_points;

    // Fill the edges
    for (unsigned int i = 0; i < polygon_vertices.size(); i++)
    {
        Point curr_point = polygon_vertices[i];
        Point next_point;

        if (i == polygon_vertices.size() - 1)
        {
            // The next point of the last point is the first point
            next_point = polygon_vertices[0];
        }
        else
        {
            next_point = polygon_vertices[i + 1];
        }

        // Skip if both points are the same
        if (curr_point == next_point)
            continue;

        int num_segments = static_cast<int>(
            std::ceil(distance(curr_point, next_point) / resolution_size));
        double dy = (next_point.y() - curr_point.y()) / num_segments;
        double dx = (next_point.x() - curr_point.x()) / num_segments;

        for (int segment = 1; segment < num_segments; segment++)
        {
            covered_points.emplace_back(
                Point(curr_point.x() + dx * segment, curr_point.y() + dy * segment));
        }
        // Don't add next_point as it will be added in the next iteration
        covered_points.emplace_back(curr_point);
    }


    // Fill inside
    auto max_point_y = [](const Point& a, const Point& b) { return a.y() < b.y(); };

    auto max_point_x = [](const Point& a, const Point& b) { return a.x() < b.x(); };

    // Calculate the highest and lowest x and y points
    double min_x =
        std::min_element(polygon_vertices.begin(), polygon_vertices.end(), max_point_x)
            ->x();
    double min_y =
        std::min_element(polygon_vertices.begin(), polygon_vertices.end(), max_point_y)
            ->y();
    double max_x =
        std::max_element(polygon_vertices.begin(), polygon_vertices.end(), max_point_x)
            ->x();
    double max_y =
        std::max_element(polygon_vertices.begin(), polygon_vertices.end(), max_point_y)
            ->y();

    int num_pixels_x = static_cast<int>(std::ceil((max_x - min_x) / resolution_size));
    int num_pixels_y = static_cast<int>(std::ceil((max_y - min_y) / resolution_size));

    for (int x_pixel = 0; x_pixel <= num_pixels_x; x_pixel++)
    {
        // x and y offset from the top left corner of the rectangle
        double x_offset;

        // Adjust the last x and y pixels to be on the edge of the rectangle to make sure
        // that the points cover the entire rectangle without going outside.
        if (x_pixel == num_pixels_x)
        {
            x_offset = max_x - min_x - FIXED_EPSILON;
        }
        else
        {
            x_offset = x_pixel * resolution_size;
        }
        double x_coord = min_x + x_offset + FIXED_EPSILON;

        for (int y_pixel = 0; y_pixel <= num_pixels_y; y_pixel++)
        {
            double y_offset;
            if (y_pixel == num_pixels_y)
            {
                y_offset = max_y - min_y - FIXED_EPSILON;
            }
            else
            {
                y_offset = y_pixel * resolution_size;
            }
            double y_coord = min_y + y_offset + FIXED_EPSILON;

            Point curr_point(x_coord, y_coord);
            if (contains(polygon, curr_point))
            {
                covered_points.emplace_back(curr_point);
            }
        }
    }

    return covered_points;
}

std::vector<Point> rasterize(const Stadium& stadium, double resolution_size)
{
    std::vector<Point> covered_points;
    std::vector<Point> start_circle = rasterize(
        Circle(stadium.segment().getStart(), stadium.radius()), resolution_size);
    std::vector<Point> end_circle =
        rasterize(Circle(stadium.segment().getEnd(), stadium.radius()), resolution_size);

    // construct rectangle polygon
    Polygon inner_rectangle = stadium.innerRectangle();

    std::vector<Point> rectangle_points = rasterize(inner_rectangle, resolution_size);

    covered_points.insert(covered_points.end(), start_circle.begin(), start_circle.end());
    covered_points.insert(covered_points.end(), end_circle.begin(), end_circle.end());
    covered_points.insert(covered_points.end(), rectangle_points.begin(),
                          rectangle_points.end());
    return covered_points;
}
