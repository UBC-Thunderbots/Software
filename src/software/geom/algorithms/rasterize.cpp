#include "software/geom/algorithms/rasterize.h"

#include <algorithm>
#include <cmath>

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersection.h"
#include "software/geom/algorithms/rasterize.h"


bool isInPixel(const Point& a, const Point& b, double resolution_size);
std::optional<Point> isAVertex(const Point& point, const Polygon& polygon, double resolution_size);
const Point& find_next_point_in_polygon(const Point& point, const Polygon& polygon);
const Point& find_previous_point_in_polygon(const Point& point, const Polygon& polygon);
bool is_special_vertex(const Point& point, const Polygon& polygon, const double resolution_size);

std::vector<Point> rasterize(const Circle& circle, const double resolution_size)
{
    std::vector<Point> covered_points;

    // Using an approach to find the points on the edges using y = +-sqrt(r^2 - (x - k)^2)
    // + h and filling in the rest Downside is using sqrt to calculate Chosen to avoid
    // points being outside of circle due to floating point errors
    const double EPSILON = 0.0001;
    double radius        = circle.radius() - EPSILON;
    double diameter      = radius * 2;
    Point origin         = circle.origin();

    // max number of pixels in each dimension
    int max_num_pixels = (int)std::ceil(diameter / resolution_size);

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

        int y_num_pixels = (int)std::ceil((y_max - y_min) / resolution_size);
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

    return covered_points;
}

std::vector<Point> rasterize(const Rectangle& rectangle, const double resolution_size)
{
    std::vector<Point> covered_points;

    int num_pixels_x = (int)std::ceil(rectangle.xLength() / resolution_size);
    int num_pixels_y = (int)std::ceil(rectangle.yLength() / resolution_size);

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
    // Used to avoid points being outside of polygon due to floating point errors
    const double EPSILON         = 0.0001;
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

        int num_segments =
            (int)std::ceil(distance(curr_point, next_point) / resolution_size);
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

    int num_pixels_x = (int)std::ceil((max_x - min_x) / resolution_size);
    int num_pixels_y = (int)std::ceil((max_y - min_y) / resolution_size);

    for (int x_pixel = 0; x_pixel <= num_pixels_x; x_pixel++)
    {
        // x and y offset from the top left corner of the rectangle
        double x_offset;

        // Adjust the last x and y pixels to be on the edge of the rectangle to make sure
        // that the points cover the entire rectangle without going outside.
        if (x_pixel == num_pixels_x)
        {
            x_offset = max_x - min_x - EPSILON;
        }
        else
        {
            x_offset = x_pixel * resolution_size;
        }
        double x_coord = min_x + x_offset + EPSILON;

        for (int y_pixel = 0; y_pixel <= num_pixels_y; y_pixel++)
        {
            double y_offset;
            if (y_pixel == num_pixels_y)
            {
                y_offset = max_y - min_y - EPSILON;
            }
            else
            {
                y_offset = y_pixel * resolution_size;
            }
            double y_coord = min_y + y_offset + EPSILON;

            Point curr_point(x_coord, y_coord);
            if (contains(polygon, curr_point))
            {
                covered_points.emplace_back(curr_point);
            }
        }
    }

    return covered_points;
}

bool isInPixel(const Point& a, const Point& b, double resolution_size)
{
    double min_x = a.x() - resolution_size / 2;
    double min_y = a.y() - resolution_size / 2;
    double max_x = a.x() + resolution_size / 2;
    double max_y = a.y() + resolution_size / 2;

    double b_x = b.x();
    double b_y = b.y();
    return (b_x >= min_x && b_x <= max_x) && (b_y >= min_y && b_y <= max_y);
}

std::optional<Point> isAVertex(const Point& point, const Polygon& polygon, double resolution_size)
{
    const auto& polygon_vertices = polygon.getPoints();
    for (auto i = polygon_vertices.begin(); i != polygon_vertices.end(); ++i)
    {
        if (isInPixel(point, *i, resolution_size))
        {
            return std::optional<Point>(*i);
        }
    }
    return std::nullopt;
}

const Point& find_next_point_in_polygon(const Point& point, const Polygon& polygon)
{
    const auto& polygon_vertices = polygon.getPoints();
    auto index = std::find(polygon_vertices.begin(), polygon_vertices.end(), point) - polygon_vertices.begin();

    if ((unsigned) index == (polygon_vertices.size() - 1))
    {
        return polygon_vertices[0];
    }
    else
    {
        return polygon_vertices[index + 1];
    }
}

const Point& find_previous_point_in_polygon(const Point& point, const Polygon& polygon)
{
    const auto& polygon_vertices = polygon.getPoints();
    auto index = std::find(polygon_vertices.begin(), polygon_vertices.end(), point) - polygon_vertices.begin();

    if (index == 0)
    {
        return polygon_vertices[polygon_vertices.size() - 1];
    }
    else
    {
        return polygon_vertices[index - 1];
    }
}

bool is_special_vertex(const Point& point, const Polygon& polygon, const double resolution_size)
{
    std::optional<Point> vertex = isAVertex(point, polygon, resolution_size);
    if(vertex == std::nullopt)
    {
        return false;
    }

    Point prev_point = find_previous_point_in_polygon(vertex.value(), polygon);
    Point next_point = find_next_point_in_polygon(vertex.value(), polygon);

    return ((prev_point.y() > point.y() && (next_point.y() > point.y()))
            || ((prev_point.y() < point.y()) && (next_point.y() < point.y())));
}
