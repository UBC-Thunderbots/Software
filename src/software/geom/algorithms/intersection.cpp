#include "software/geom/algorithms/intersection.h"

#include "software/geom/algorithms/almost_equal.h"
#include "software/geom/algorithms/collinear.h"
#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/convex_angle.h"

std::optional<Point> intersection(const Point &a, const Point &b, const Point &c,
                                  const Point &d, double fixed_epsilon, int ulps_epsilon)
{
    double x1 = a.x();
    double y1 = a.y();
    double x2 = b.x();
    double y2 = b.y();
    double x3 = c.x();
    double y3 = c.y();
    double x4 = d.x();
    double y4 = d.y();

    double denominatorTermA = (x1 - x2) * (y3 - y4);
    double denominatorTermB = (y1 - y2) * (x3 - x4);
    double denominator      = denominatorTermA - denominatorTermB;

    if (almostEqual(denominatorTermA, denominatorTermB, fixed_epsilon, ulps_epsilon))
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
    if (first == second)
    {
        return {first.getStart(), first.getEnd()};
    }

    std::vector<Point> intersections;

    Point a = first.getStart();
    Point b = first.getEnd();
    Point c = second.getStart();
    Point d = second.getEnd();

    // check for overlaps
    if (contains(second, a))
    {
        intersections.emplace_back(a);
    }
    if (contains(second, b))
    {
        intersections.emplace_back(b);
    }
    if (contains(first, c))
    {
        intersections.emplace_back(c);
    }
    if (contains(first, d))
    {
        intersections.emplace_back(d);
    }
    if (intersections.size() > 0)
    {
        return intersections;
    }

    auto intersection_value = intersection(a, b, c, d);
    if (intersection_value)
    {
        if (contains(first, *intersection_value) && contains(second, *intersection_value))
        {
            intersections.emplace_back(*intersection_value);
        }
    }

    return intersections;
}

std::unordered_set<Point> intersection(const Polygon &polygon, const Segment &segment)
{
    std::unordered_set<Point> intersections;

    for (const Segment &seg : polygon.getSegments())
    {
        for (const Point &p : intersection(seg, segment))
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
        intersection(ray.getStart(), second, segment.getStart(), segment.getEnd());

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
             collinear(ray.getStart(), segment.getStart(), segment.getEnd()))
    {
        // Check if ray passes through both segment start and end
        if (ray.toUnitVector() == (segment.getStart() - ray.getStart()).normalize() &&
            ray.toUnitVector() == (segment.getEnd() - ray.getStart()).normalize())
        {
            intersections = {segment.getStart(), segment.getEnd()};
            return intersections;
        }
        // Ray origin within the segment, return the ray start position and the segment
        // endpoint in the ray's direction
        else
        {
            Point overlapping_segment_end =
                ray.toUnitVector() == (segment.getEnd() - segment.getStart()).normalize()
                    ? segment.getEnd()
                    : segment.getStart();
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

std::unordered_set<Point> intersection(const Polygon &polygon, const Ray &ray)
{
    std::unordered_set<Point> intersections;

    for (const Segment &seg : polygon.getSegments())
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

    if (convexAngle(intersection_first_direction, first.toUnitVector()) <
            Angle::quarter() &&
        convexAngle(intersection_second_direction, second.toUnitVector()) <
            Angle::quarter())
    {
        return point_of_intersection.value();
    }
    else
    {
        return std::nullopt;
    }
}
