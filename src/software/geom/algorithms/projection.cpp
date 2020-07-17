#include "software/geom/algorithms/projection.h"

#include "software/geom/algorithms/contains.h"
#include "software/geom/algorithms/multiple_segments.h"
#include "software/geom/algorithms/projection_impl.h"

std::vector<Segment> projectCirclesOntoSegment(const Segment &segment,
                                               const std::vector<Circle> &circles,
                                               const Point &origin)
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
