#include "software/geom/algorithms/step_along_perimeter.h"

#include <algorithm>
#include <vector>

#include "software/geom/algorithms/collinear.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/segment.h"


Point stepAlongPerimeter(const Polygon& polygon, const Point& start,
                         double travel_distance)
{
    if (travel_distance == 0.0)
    {
        return start;
    }

    const std::vector<Segment>& polygon_segments = polygon.getSegments();
    std::size_t start_segment_index              = 0;

    // find initial segment which contains start point
    auto it =
        std::find_if(polygon_segments.begin(), polygon_segments.end(),
                     [&start](const Segment& segment) {
                         return collinear(segment.getStart(), start, segment.getEnd());
                     });

    if (it == polygon_segments.end()) {
        throw std::runtime_error("Point not on polygon");
    }

    start_segment_index = std::distance(polygon_segments.begin(), it);

    std::size_t segment_index = start_segment_index;

    // fmod travel distance for case where travel_distance > perimeter
    double perimeter = polygon.perimeter();
    bool is_counter_clockwise = travel_distance < 0;
    travel_distance           = std::fmod(std::abs(travel_distance), perimeter);
    if (is_counter_clockwise)
    {
        travel_distance = perimeter - travel_distance;
    }

    bool wrap_flag = false;
    while (travel_distance > 0)
    {
        Segment curr_segment = polygon_segments[segment_index];

        double segment_length = curr_segment.length();
        if (segment_index == start_segment_index && !wrap_flag)
        {
            segment_length = distance(start, curr_segment.getEnd());
            wrap_flag      = true;
        }

        // If the remaining distance to travel is less than or equal to the length
        // of the current segment, calculate the final point and return it
        if (travel_distance <= segment_length)
        {
            double ratio = travel_distance / segment_length;
            double newX =
                curr_segment.getStart().x() +
                ratio * (curr_segment.getEnd().x() - curr_segment.getStart().x());
            double newY =
                curr_segment.getStart().y() +
                ratio * (curr_segment.getEnd().y() - curr_segment.getStart().y());
            return Point{newX, newY};
        }

        // Subtract the length of the current segment from the total distance
        travel_distance -= segment_length;

        // Update the segment index
        segment_index = (segment_index + 1) % polygon_segments.size();
    }

    return start;
}
