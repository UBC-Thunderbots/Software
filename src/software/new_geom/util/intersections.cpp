#include "software/new_geom/util/intersections.h"

std::vector<Point> intersection(const Segment &first, const Segment &second)
{
    std::vector<Point> output;

    boost::geometry::model::segment<Point> AB(first.getSegStart(), first.getEnd());
    boost::geometry::model::segment<Point> CD(second.getSegStart(), second.getEnd());
    boost::geometry::intersection(AB, CD, output);

    return output;
}

std::vector<Point> intersection(const Rectangle &rectangle, const Segment &segment)
{
    std::vector<Point> intersections;

    for (const Segment &recSegment : rectangle.getSegments())
    {
        for (const Point &p : intersection(recSegment, segment))
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

    std::optional<Point> pointOfIntersection =
        intersection(ray.getStart(), second, segment.getSegStart(), segment.getEnd());

    // If there exists a single intersection, and it exists on the ray and within the
    // segment
    if (pointOfIntersection.has_value() && contains(ray, pointOfIntersection.value()) &&
        contains(segment, pointOfIntersection.value()))
    {
        intersections = {pointOfIntersection.value()};
        return intersections;
    }
    // If no intersection was found and the ray and segment are parallel, and collinear
    else if (!pointOfIntersection.has_value() &&
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
            Point overlappingSegmentEnd =
                ray.toUnitVector() ==
                        (segment.getEnd() - segment.getSegStart()).normalize()
                    ? segment.getEnd()
                    : segment.getSegStart();
            intersections = {ray.getStart(), overlappingSegmentEnd};
            return intersections;
        }
    }
    // The ray and segment do not intersect at all
    else
    {
        return intersections;
    }
}

// From:
// https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#Given_two_points_on_each_line
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
    std::optional<Point> pointOfIntersection =
        intersection(first.getStart(), first.getStart() + first.toUnitVector(),
                     second.getStart(), second.getStart() + second.toUnitVector());

    if (!pointOfIntersection.has_value())
    {
        return std::nullopt;
    }

    // Check if the intersection exits along the direction of both rays
    Vector intersection_first_direction =
        (pointOfIntersection.value() - first.getStart());
    Vector intersection_second_direction =
        (pointOfIntersection.value() - second.getStart());

    if (sign(intersection_first_direction.x()) == sign(first.toUnitVector().x()) &&
        sign(intersection_first_direction.y()) == sign(first.toUnitVector().y()) &&
        sign(intersection_second_direction.x()) == sign(second.toUnitVector().x()) &&
        sign(intersection_second_direction.y()) == sign(second.toUnitVector().y()))
    {
        return pointOfIntersection.value();
    }
    else
    {
        return std::nullopt;
    }
}
