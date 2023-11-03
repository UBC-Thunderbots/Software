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
    std::size_t start_segment_index;

    //finds the closest segment on the polygon to the start point with the maximum distance being 0.05
//    const double MAX_DISTANCE = 0.05; // arbitrary number, lmk if you need to change
//    auto min_it = std::find_if(polygon_segments.begin(), polygon_segments.end(),
//                               [&start, MAX_DISTANCE](const auto& segment) {
//                                   double dist = distance(start, segment);
//                                   return dist < MAX_DISTANCE;
//                               }
//    );
//
//    if (min_it != polygon_segments.end()) {
//        start_segment_index = std::distance(polygon_segments.begin(), min_it);
//    } else {
//        // Handling the case where no segment is within the upper_limit,
//        throw std::runtime_error("Point not on polygon");
//    }


//     Implementation in case no upper limit needs to be imposed
    // finds the closest segment to start point
    const double MAX_DISTANCE = 0.05; // arbitrary number, lmk if you need to change
    auto min_it = std::min_element(polygon_segments.begin(), polygon_segments.end(),
                                   [&start](const auto& a, const auto& b) {
                                       return distance(start, a) < distance(start, b);
                                   }
    );
    start_segment_index = std::distance(polygon_segments.begin(), min_it);

    if (distance(start, polygon_segments[start_segment_index]) > MAX_DISTANCE) {
        throw std::runtime_error("Point not on polygon");
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
