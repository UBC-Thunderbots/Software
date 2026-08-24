#include "software/geom/algorithms/step_along_perimeter.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "software/geom/algorithms/closest_point.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/geom_constants.h"
#include "software/geom/segment.h"


Point stepAlongPerimeter(const Polygon& polygon, const Point& start,
                         double travel_distance)
{
    const std::vector<Segment>& polygon_segments = polygon.getSegments();

    auto min_it = std::min_element(polygon_segments.begin(), polygon_segments.end(),
                                   [&start](const auto& a, const auto& b)
                                   { return distance(start, a) < distance(start, b); });
    std::size_t start_segment_index = std::distance(polygon_segments.begin(), min_it);

    // finds the point closest to start point on the segment
    Point closest_start = closestPoint(start, polygon_segments[start_segment_index]);

    if (std::abs(travel_distance) < FIXED_EPSILON)
    {
        return closest_start;
    }

    std::size_t segment_index = start_segment_index;

    // fmod travel distance for case where travel_distance > perimeter
    double perimeter          = polygon.perimeter();
    bool is_counter_clockwise = travel_distance < 0;
    travel_distance           = std::fmod(std::abs(travel_distance), perimeter);
    if (is_counter_clockwise)
    {
        travel_distance = perimeter - travel_distance;
    }

    // First segment is traversed from closest_start toward the segment end; later
    // segments are traversed from their start vertex.
    bool on_first_segment = true;
    while (travel_distance > FIXED_EPSILON)
    {
        Segment curr_segment  = polygon_segments[segment_index];
        Point from_point      = curr_segment.getStart();
        double segment_length = curr_segment.length();

        if (on_first_segment)
        {
            from_point       = closest_start;
            segment_length   = distance(closest_start, curr_segment.getEnd());
            on_first_segment = false;
        }

        if (segment_length < FIXED_EPSILON)
        {
            // End of segment. Advance to next.
            segment_index = (segment_index + 1) % polygon_segments.size();
            continue;
        }

        if (travel_distance <= segment_length)
        {
            double ratio = travel_distance / segment_length;
            return Point(
                from_point.x() + ratio * (curr_segment.getEnd().x() - from_point.x()),
                from_point.y() + ratio * (curr_segment.getEnd().y() - from_point.y()));
        }

        travel_distance -= segment_length;
        segment_index = (segment_index + 1) % polygon_segments.size();
    }

    return closest_start;
}
