#include "software/geom/algorithms/projection_impl.h"

#include <vector>

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/intersection.h"

std::optional<Segment> segmentEnclosedBetweenRays(const Segment &segment, const Ray &ray1,
                                                  const Ray &ray2)
{
    // Create rays located at the extremes of the segment, that point in the direction
    // outwards are parallel to the segment
    const Ray extremes1 =
        Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getStart()));
    const Ray extremes2 =
        Ray(segment.getStart(), Vector(segment.getStart() - segment.getEnd()));

    const std::optional<Point> extreme_intersect11 = intersection(extremes1, ray1);
    const std::optional<Point> extreme_intersect12 = intersection(extremes2, ray1);
    const std::optional<Point> extreme_intersect21 = intersection(extremes1, ray2);
    const std::optional<Point> extreme_intersect22 = intersection(extremes2, ray2);

    // Check for the cases that the rays intersect the same segment projection
    if ((extreme_intersect11.has_value() && extreme_intersect21.has_value()) ||
        (extreme_intersect12.has_value() && extreme_intersect22.has_value()))
    {
        return std::nullopt;
    }
    else
    {
        // Since we know that both rays aren't passing through the same side of the
        // segment at this point, then as long as they both only intersect 1 point the
        // segment must be enclosed between them
        if ((extreme_intersect11.has_value() != extreme_intersect12.has_value()) &&
            (extreme_intersect21.has_value() != extreme_intersect22.has_value()))
        {
            return std::make_optional(segment);
        }
        // Covers the case where a single ray passes by both sides of the segment
        else
        {
            return std::nullopt;
        }
    }
}

std::optional<Segment> getIntersectingSegment(const Ray &ray1, const Ray &ray2,
                                              const Segment &segment)
{
    // Check if the segment is enclosed between the rays
    if (segmentEnclosedBetweenRays(segment, ray1, ray2))
    {
        return segment;
    }

    // Calculate intersections of each individual ray and the segment
    std::vector<Point> intersection1 = intersection(ray1, segment);
    std::vector<Point> intersection2 = intersection(ray2, segment);

    std::optional<Point> intersect11;
    std::optional<Point> intersect12;
    std::optional<Point> intersect21;
    std::optional<Point> intersect22;

    if (!intersection1.empty())
    {
        intersect11 = intersection1[0];

        if (intersection1.size() > 1)
        {
            intersect12 = intersection1[1];
        }
    }

    if (!intersection2.empty())
    {
        intersect21 = intersection2[0];

        if (intersection2.size() > 1)
        {
            intersect22 = intersection2[1];
        }
    }

    // Check if there are any real intersections
    if (!intersect11.has_value() && !intersect21.has_value())
    {
        return std::nullopt;
    }
    // Check if one of the rays is overlapping the segment. If this is the case, return
    // the segment (If a ray intersects a ray more than one time it must be overlapping)
    else if ((intersect11.has_value() && intersect12.has_value()) ||
             (intersect21.has_value() && intersect22.has_value()))
    {
        return segment;
    }
    // If there is only one intersection point for each ray combine the intersections into
    // a segment
    else if ((intersect11.has_value() && !intersect12.has_value()) &&
             (intersect21.has_value() && !intersect22.has_value()))
    {
        return std::make_optional(Segment(intersect11.value(), intersect21.value()));
    }
    // If only one ray intersects the segment return the segment between the intersection
    // and the segment extreme Point (intersection11 is real, intersection22 is not)
    else if (intersect11.has_value() && !intersect21.has_value())
    {
        const Ray extremes1 =
            Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getStart()));
        const Ray extremes2 =
            Ray(segment.getStart(), Vector(segment.getStart() - segment.getEnd()));
        ;

        std::optional<Point> extreme_intersect1 = intersection(extremes1, ray2);
        std::optional<Point> extreme_intersect2 = intersection(extremes2, ray2);

        if (extreme_intersect1.has_value())
        {
            return std::make_optional(Segment(intersect11.value(), segment.getEnd()));
        }
        else if (extreme_intersect2.has_value())
        {
            return std::make_optional(Segment(intersect11.value(), segment.getStart()));
        }
    }
    // If only one ray intersects the segment return the segment between the intersection
    // and the segment extreme (intersection11 is real, intersection22 is not)
    else if (intersect21.has_value() && !intersect11.has_value())
    {
        const Ray extremes1 =
            Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getStart()));
        const Ray extremes2 =
            Ray(segment.getStart(), Vector(segment.getStart() - segment.getEnd()));
        ;

        std::optional<Point> extreme_intersect1 = intersection(extremes1, ray1);
        std::optional<Point> extreme_intersect2 = intersection(extremes2, ray1);

        if (extreme_intersect1.has_value())
        {
            return std::make_optional(Segment(intersect21.value(), segment.getEnd()));
        }
        else if (extreme_intersect2.has_value())
        {
            return std::make_optional(Segment(intersect21.value(), segment.getStart()));
        }
    }
    // All cases have been checked, return std::nullopt
    return std::nullopt;
}

std::pair<Point, Point> getCircleTangentPoints(const Point &start, const Circle &circle)
{
    // If the point is already inside the circe arccos won't work so just return
    // the perp points
    if (contains(circle, start))
    {
        double perpDist = std::sqrt(circle.radius() * circle.radius() -
                                    (circle.origin() - start).lengthSquared());
        Point p1 = start + (circle.origin() - start).perpendicular().normalize(perpDist);
        Point p2 =
            start - ((circle.origin() - start).perpendicular().normalize(perpDist));
        return std::make_pair(p1, p2);
    }
    else
    {
        double radiusAngle =
            std::acos(circle.radius() / (start - circle.origin()).length());
        Point p1 = circle.origin() + (start - circle.origin())
                                         .rotate(Angle::fromRadians(radiusAngle))
                                         .normalize(circle.radius());
        Point p2 = circle.origin() + (start - circle.origin())
                                         .rotate(-Angle::fromRadians(radiusAngle))
                                         .normalize(circle.radius());
        return std::make_pair(p1, p2);
    }
}

std::pair<Ray, Ray> getCircleTangentRaysWithReferenceOrigin(const Point &reference,
                                                            const Circle &circle)
{
    auto [tangent_point1, tangent_point2] = getCircleTangentPoints(reference, circle);

    return std::make_pair(Ray(reference, (tangent_point1 - reference)),
                          Ray(reference, (tangent_point2 - reference)));
}
