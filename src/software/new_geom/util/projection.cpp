#include "software/new_geom/util/projection.h"

std::pair<Point, Point> getCircleTangentPoints(const Point &start, const Circle &circle,
                                               double buffer)
{
    // If the point is already inside the circe arccos won't work so just return
    // the perp points
    if (contains(circle, start))
    {
        double perpDist = std::sqrt(circle.getRadius() * circle.getRadius() -
                                    (circle.getOrigin() - start).lengthSquared());
        Point p1 =
            start +
            (circle.getOrigin() - start).perpendicular().normalize(perpDist + buffer);
        Point p2 =
            start -
            ((circle.getOrigin() - start).perpendicular().normalize(perpDist + buffer));
        return std::make_pair(p1, p2);
    }
    else
    {
        double radiusAngle =
            std::acos(circle.getRadius() / (start - circle.getOrigin()).length());
        Point p1 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(Angle::fromRadians(radiusAngle))
                                            .normalize(circle.getRadius() + buffer);
        Point p2 = circle.getOrigin() + (start - circle.getOrigin())
                                            .rotate(-Angle::fromRadians(radiusAngle))
                                            .normalize(circle.getRadius() + buffer);
        return std::make_pair(p1, p2);
    }
}

std::pair<Ray, Ray> getCircleTangentRaysWithReferenceOrigin(const Point reference,
                                                            const Circle circle)
{
    auto [tangent_point1, tangent_point2] = getCircleTangentPoints(reference, circle, 0);

    return std::make_pair(Ray(reference, (tangent_point1 - reference)),
                          Ray(reference, (tangent_point2 - reference)));
}

std::optional<Segment> segmentEnclosedBetweenRays(Segment segment, Ray ray1, Ray ray2)
{
    // Create rays located at the extremes of the segment, that point in the direction
    // outwards are parallel to the segment
    const Ray extremes1 =
        Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getSegStart()));
    const Ray extremes2 =
        Ray(segment.getSegStart(), Vector(segment.getSegStart() - segment.getEnd()));

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
std::optional<Segment> getIntersectingSegment(Ray ray1, Ray ray2, Segment segment)
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
            Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getSegStart()));
        const Ray extremes2 =
            Ray(segment.getSegStart(), Vector(segment.getSegStart() - segment.getEnd()));
        ;

        std::optional<Point> extreme_intersect1 = intersection(extremes1, ray2);
        std::optional<Point> extreme_intersect2 = intersection(extremes2, ray2);

        if (extreme_intersect1.has_value())
        {
            return std::make_optional(Segment(intersect11.value(), segment.getEnd()));
        }
        else if (extreme_intersect2.has_value())
        {
            return std::make_optional(
                Segment(intersect11.value(), segment.getSegStart()));
        }
    }
    // If only one ray intersects the segment return the segment between the intersection
    // and the segment extreme (intersection11 is real, intersection22 is not)
    else if (intersect21.has_value() && !intersect11.has_value())
    {
        const Ray extremes1 =
            Ray(segment.getEnd(), Vector(segment.getEnd() - segment.getSegStart()));
        const Ray extremes2 =
            Ray(segment.getSegStart(), Vector(segment.getSegStart() - segment.getEnd()));
        ;

        std::optional<Point> extreme_intersect1 = intersection(extremes1, ray1);
        std::optional<Point> extreme_intersect2 = intersection(extremes2, ray1);

        if (extreme_intersect1.has_value())
        {
            return std::make_optional(Segment(intersect21.value(), segment.getEnd()));
        }
        else if (extreme_intersect2.has_value())
        {
            return std::make_optional(
                Segment(intersect21.value(), segment.getSegStart()));
        }
    }
    // All cases have been checked, return std::nullopt
    return std::nullopt;
}

std::optional<Segment> mergeOverlappingParallelSegments(Segment segment1,
                                                        Segment segment2)
{
    std::optional<Segment> redundant_segment =
        mergeFullyOverlappingSegments(segment1, segment2);

    // If the segments are not parallel, then return std::nullopt. (The segments are
    // parallel of all points are collinear)
    if (!collinear(segment1, segment2))
    {
        return std::nullopt;
    }
    // Check the case where one segment is completely contained in the other
    else if (redundant_segment.has_value())
    {
        return redundant_segment;
    }
    // Check if the beginning of segment2 lays inside segment1
    else if (contains(segment1, segment2.getSegStart()))
    {
        // If segment2.getSegStart() lays in segment1, then the combined segment is
        // segment2,getEnd() and the point furthest from segment2.getEnd()
        return (segment1.getSegStart() - segment2.getEnd()).lengthSquared() >
                       (segment1.getEnd() - segment2.getEnd()).lengthSquared()
                   ? Segment(segment1.getSegStart(), segment2.getEnd())
                   : Segment(segment1.getEnd(), segment2.getEnd());
    }
    // Now check if the end of segment2 lays inside segment1
    else if (contains(segment1, segment2.getEnd()))
    {
        // If segment2.getSegStart() lays in segment1, then the combined segment is
        // segment2,getEnd() and the point furtherst from segmen2.getEnd()
        return (segment1.getSegStart() - segment2.getSegStart()).lengthSquared() >
                       (segment1.getEnd() - segment2.getSegStart()).lengthSquared()
                   ? Segment(segment1.getSegStart(), segment2.getSegStart())
                   : Segment(segment1.getEnd(), segment2.getSegStart());
    }
    return std::nullopt;
}

std::optional<Segment> mergeFullyOverlappingSegments(Segment segment1, Segment segment2)
{
    // If the segments are not parallel, then return std::nullopt. (The segments are
    // parallel if all points are collinear)
    if (!collinear(segment1, segment2))
    {
        return std::nullopt;
    }

    Segment largest_segment, smallest_segment;
    // Grab the largest segment
    if (segment1.toVector().lengthSquared() > segment2.toVector().lengthSquared())
    {
        largest_segment  = segment1;
        smallest_segment = segment2;
    }
    else
    {
        largest_segment  = segment2;
        smallest_segment = segment1;
    }

    // The segment is redundant if both points of the smallest segment are contained in
    // the largest segment
    if (contains(largest_segment, smallest_segment.getSegStart()) &&
        contains(largest_segment, smallest_segment.getEnd()))
    {
        return std::make_optional(largest_segment);
    }
    else
    {
        return std::nullopt;
    }
}

std::vector<Segment> getEmptySpaceWithinParentSegment(std::vector<Segment> segments,
                                                      Segment parent_segment)
{
    // Make sure the starting point of all segments is closer to the start of the
    // reference segment to simplify the evaluation
    for (auto &unordered_seg : segments)
    {
        if ((parent_segment.getSegStart() - unordered_seg.getSegStart()).length() >
            (parent_segment.getSegStart() - unordered_seg.getEnd()).length())
        {
            // We need to flip the start/end of the segment
            Segment temp = unordered_seg;
            unordered_seg.setSegStart(temp.getEnd());
            unordered_seg.setEnd(temp.getSegStart());
        }
    }

    // Now we must sort the segments so that we can iterate through them in order to
    // generate open angles sort using a lambda expression
    // We sort the segments based on how close their 'start' point is to the 'start'
    // of the reference Segment
    std::sort(segments.begin(), segments.end(), [parent_segment](Segment &a, Segment &b) {
        return (parent_segment.getSegStart() - a.getSegStart()).length() <
               (parent_segment.getSegStart() - b.getSegStart()).length();
    });

    // Now we need to find the largest open segment/angle
    std::vector<Segment> open_segs;

    // The first Angle is between the reference Segment and the first obstacle Segment
    // After this one, ever open angle is between segment(i).end and
    // segment(i+1).start
    open_segs.push_back(
        Segment(parent_segment.getSegStart(), segments.front().getSegStart()));

    // The 'open' Segment in the space between consecutive 'blocking' Segments
    for (std::vector<Segment>::const_iterator it = segments.begin();
         it != segments.end() - 1; it++)
    {
        open_segs.push_back(Segment(it->getEnd(), (it + 1)->getSegStart()));
    }

    // Lastly, the final open angle is between obstacles.end().getEnd() and
    // reference_segment.getEnd()
    open_segs.push_back(Segment(segments.back().getEnd(), parent_segment.getEnd()));

    // Remove all zero length open Segments
    for (std::vector<Segment>::const_iterator it = open_segs.begin();
         it != open_segs.end();)
    {
        // 2 * FIXED_EPSILON for error needed:
        // - segment subtraction (end - start)
        // - mathcalls `hypot` from `length` function call
        if (it->length() < 2 * FIXED_EPSILON)
        {
            open_segs.erase(it);
        }
        else
        {
            it++;
        }
    }

    return open_segs;
}


std::vector<Segment> projectCirclesOntoSegment(Segment segment,
                                               std::vector<Circle> circles, Point origin)
{
    // Loop through all obstacles to create their projected Segment
    std::vector<Segment> obstacle_segment_projections = {};

    for (Circle circle : circles)
    {
        // If the reference is inside an obstacle there is no open direction
        if (contains(circle, origin))
        {
            obstacle_segment_projections.push_back(segment);
            return obstacle_segment_projections;
        }

        // Get the tangent rays from the reference point to the obstacle
        auto [ray1, ray2] = getCircleTangentRaysWithReferenceOrigin(origin, circle);

        // Project the tangent Rays to obtain a 'blocked' segment on the reference
        // Segment
        std::optional<Segment> intersect_segment =
            getIntersectingSegment(ray1, ray2, segment);

        if (intersect_segment.has_value())
        {
            obstacle_segment_projections.push_back(intersect_segment.value());
        }
    }
    return obstacle_segment_projections;
}

std::vector<Segment> projectSegmentsOntoVector(std::vector<Segment> segments,
                                               Vector direction)
{
    std::vector<Segment> projected_segments = {};


    // Project all Segments onto the direction Vector
    for (Segment segment : segments)
    {
        // The projection of the Segment without including the original Segment location
        Vector raw_projection = segment.toVector().project(direction);

        // Only count projections that have a non-zero magnitude
        if (raw_projection.lengthSquared() > FIXED_EPSILON)
        {
            projected_segments.push_back(
                Segment(segment.getSegStart(), segment.getSegStart() + raw_projection));
        }
    }
    std::vector<Segment> unique_segments;

    unsigned int j = 0;
    // Loop through all segments and combine segments
    // to reduce the vector to the smallest number of independent (not overlapping)
    // segments
    while (projected_segments.size() > 0)
    {
        std::optional<Segment> temp_segment;
        unique_segments.push_back(projected_segments[0]);
        projected_segments.erase(projected_segments.begin());

        for (unsigned int i = 0; i < projected_segments.size(); i++)
        {
            temp_segment = mergeOverlappingParallelSegments(unique_segments[j],
                                                            projected_segments[i]);

            if (temp_segment.has_value())
            {
                unique_segments[j] = temp_segment.value();
                // Remove segments[i] from the list as it is not unique
                projected_segments.erase(projected_segments.begin() + i);
                i--;
            }
        }
        j++;
    }

    return unique_segments;
}
