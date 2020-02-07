#include "software/new_geom/util/intersection.h"
#include "software/new_geom/util/contains.h"
#include "software/new_geom/util/collinear.h"

/**
 * Computes the point of intersection between two lines.
 * Note: this computes the intersection of two lines, not line segments.
 *
 * Overlapping lines are considered parallel and not intersecting.
 *
 * See:
 * https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
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

    double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
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

/**
 * Returns the sign of the given double, or zero if is in the range (-EPSILON, EPSILON).
 *
 * @param n the given double
 *
 * @return the sign of the given double, or zero if is in the range (-EPSILON, EPSILON)
 */
constexpr int sign(double n)
{
    return n > GeomConstants::EPSILON ? 1 : (n < -GeomConstants::EPSILON ? -1 : 0);
}

std::vector<Point> intersection(const Segment &first, const Segment &second)
{
    std::vector<Point> output;

    boost::geometry::model::segment<Point> seg_1(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> seg_2(second.getSegStart(), second.getEnd());
    boost::geometry::intersection(seg_1, seg_2, output);

    return output;
}

std::unordered_set<Point> intersection(const Rectangle &rectangle, const Segment &segment)
{
    std::unordered_set<Point> intersections;

    for (const Segment &rec_segment : rectangle.getSegments())
    {
        for (const Point &p : intersection(rec_segment, segment))
        {
            intersections.insert(p);
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
    double a1          = first.getCoeffs().a;
    double b1          = first.getCoeffs().b;
    double c1          = first.getCoeffs().c;
    double a2          = second.getCoeffs().a;
    double b2          = second.getCoeffs().b;
    double c2          = second.getCoeffs().c;
    double determinant = (a1 * b2) - (a2 * b1);

    if (determinant == 0)
    {
        // Lines are parallel
        return std::nullopt;
    }
    else
    {
        double x = ((b1 * c2) - (b2 * c1)) / determinant;
        double y = ((a2 * c1) - (a1 * c2)) / determinant;

        return Point(x, y);
    }
}

std::unordered_set<Point> intersection(const Rectangle &rectangle, const Ray &ray)
{
    std::unordered_set<Point> intersections;

    for (const Segment &seg : rectangle.getSegments())
    {
        for (const Point &p : intersection(ray, seg))
        {
            intersections.insert(p);
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
