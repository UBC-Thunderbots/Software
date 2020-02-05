#include "software/new_geom/util/intersection.h"

/**
 * Computes the point of intersection between two lines.
 * Note: this computes the intersection of two lines, not line segments.
 *
 * See:
 * https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
 *
 * @pre the lines are not parallel
 *
 * @params a, b points that represent the first line
 * @params c, d points that represent the second line
 *
 * @return the point of intersection, if it exists
 */
std::optional<Point> intersection(const Point &a, const Point &b, const Point &c,
                                  const Point &d)
{
    double x1 = a.x();
    double y1 = a.y();
    double x2 = b.x();
    double y2 = b.y();
    double x3 = c.x();
    double y3 = c.y();
    double x4 = d.x();
    double y4 = d.y();

    std::cout << "x1: " << x1 << std::endl;
    std::cout << "y1: " << y1 << std::endl;
    std::cout << "x2: " << x2 << std::endl;
    std::cout << "y2: " << y2 << std::endl;
    std::cout << "x3: " << x3 << std::endl;
    std::cout << "y3: " << y3 << std::endl;
    std::cout << "x4: " << x4 << std::endl;
    std::cout << "y4: " << y4 << std::endl;


    double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    std::cout << "Denom: " << denominator << std::endl;
    if (denominator == 0)
    {
        return std::nullopt;
    }

    double determinantA = x1 * y2 - y1 * x2;
    double determinantB = x3 * y4 - y3 * x4;

    Point intersection;

    intersection.set((determinantA * (x3 - x4) - (x1 - x2) * determinantB) / denominator,
                     (determinantA * (y3 - y4) - (y1 - y2) * determinantB) / denominator);

    return std::make_optional(intersection);
}

std::vector<Point> intersection(const Segment &first, const Segment &second)
{
    std::vector<Point> output;

    boost::geometry::model::segment<Point> seg_1(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> seg_2(second.getSegStart(), second.getEnd());
    boost::geometry::intersection(seg_1, seg_2, output);

    return output;
}

std::vector<Point> intersection(const Rectangle &rectangle, const Segment &segment)
{
    std::vector<Point> intersections;

    for (const Segment &rec_segment : rectangle.getSegments())
    {
        for (const Point &p : intersection(rec_segment, segment))
        {
            intersections.push_back(p);
        }
    }

    return intersections;
}

std::vector<Point> intersection(const Ray &ray, const Segment &segment)
{
    std::vector<Point> intersections;

    Point second = ray.getStart() + ray.toUnitVector();

    std::optional<Point> point_of_intersection =
        intersection(ray.getStart(), second, segment.getSegStart(), segment.getEnd());

    // If there exists a single intersection, and it exists on the ray and within the
    // segment
    if (point_of_intersection.has_value() &&
        contains(ray, point_of_intersection.value()) &&
        contains(segment, point_of_intersection.value()))
    {
        intersections = {point_of_intersection.value()};
        return intersections;
    }
    // If no intersection was found and the ray and segment are parallel, and collinear
    else if (!point_of_intersection.has_value() &&
             collinear(ray.getStart(), segment.getSegStart(), segment.getEnd()))
    {
        // Check if ray passes through both segment start and end
        if (ray.toUnitVector() == (segment.getSegStart() - ray.getStart()).normalize() &&
            ray.toUnitVector() == (segment.getEnd() - ray.getStart()).normalize())
        {
            intersections = {segment.getSegStart(), segment.getEnd()};
            return intersections;
        }
        // Ray origin within the segment, return the ray start position and the segment
        // endpoint in the ray's direction
        else
        {
            Point overlapping_segment_end =
                ray.toUnitVector() ==
                        (segment.getEnd() - segment.getSegStart()).normalize()
                    ? segment.getEnd()
                    : segment.getSegStart();
            intersections = {ray.getStart(), overlapping_segment_end};
            return intersections;
        }
    }
    // The ray and segment do not intersect at all
    else
    {
        return intersections;
    }
}

std::optional<Point> intersection(const Line &first, const Line &second)
{
    return intersection(Point(0, first.y(0)), Point(1, first.y(1)), Point(0, second.y(0)),
                        Point(1, second.y(1)));
}

std::vector<Point> intersection(const Rectangle &rectangle, const Ray &ray)
{
    std::vector<Point> intersections;

    for (const Segment &seg : rectangle.getSegments())
    {
        for (const Point &p : intersection(ray, seg))
        {
            intersections.push_back(p);
        }
    }

    return intersections;
}

std::optional<Point> intersection(const Ray &first, const Ray &second)
{
    // Calculate if an intersection exists between line representations of the rays
    std::optional<Point> point_of_intersection =
        intersection(first.getStart(), first.getStart() + first.toUnitVector(),
                     second.getStart(), second.getStart() + second.toUnitVector());

    if (!point_of_intersection.has_value())
    {
        return std::nullopt;
    }

    // Check if the intersection exits along the direction of both rays
    Vector intersection_first_direction =
        (point_of_intersection.value() - first.getStart());
    Vector intersection_second_direction =
        (point_of_intersection.value() - second.getStart());

    if (sign(intersection_first_direction.x()) == sign(first.toUnitVector().x()) &&
        sign(intersection_first_direction.y()) == sign(first.toUnitVector().y()) &&
        sign(intersection_second_direction.x()) == sign(second.toUnitVector().x()) &&
        sign(intersection_second_direction.y()) == sign(second.toUnitVector().y()))
    {
        return point_of_intersection.value();
    }
    else
    {
        return std::nullopt;
    }
}
