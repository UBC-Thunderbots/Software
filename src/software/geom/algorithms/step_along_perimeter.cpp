#include "software/geom/algorithms/step_along_perimeter.h"

#include <numeric>
#include <vector>

#include "software/geom/algorithms/collinear.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/segment.h"


Point stepAlongPerimeter(const Polygon& polygon, const Point& start,
                         double travelDistance)
{
    if (travelDistance == 0.0)
    {
        return start;
    }


    std::vector<Segment> polygonSegments = polygon.getSegments();
    std::size_t startSegmentIndex = 0;

// find initial segment which contains start point
    std::vector<Segment>::iterator it = std::find_if(polygonSegments.begin(), polygonSegments.end(),
                                                     [&start](const Segment& segment) {
                                                         return collinear(segment.getStart(), start, segment.getEnd());
                                                     });

    startSegmentIndex = std::distance(polygonSegments.begin(), it);


    std::size_t segmentIndex = startSegmentIndex;

    // fmod travel distance for case where travelDistance > perimeter
    bool isCounterClockwise = travelDistance < 0;
    travelDistance = std::fmod(std::abs(travelDistance), polygon.perimeter());
    if (isCounterClockwise) {
        travelDistance = polygon.perimeter() - travelDistance;
    }

    bool wrapFlag = false;
    while (travelDistance > 0)
    {
        Segment currSegment = polygonSegments[segmentIndex];

        double segmentLength = currSegment.length();
        if (segmentIndex == startSegmentIndex && !wrapFlag) {
            segmentLength = distance(start, currSegment.getEnd());
            wrapFlag = true;
        }

        // If the remaining distance to travel is less than or equal to the length
        // of the current segment, calculate the final point and return it
        if (travelDistance <= segmentLength)
        {
            double ratio = travelDistance / segmentLength;
            double newX  = currSegment.getStart().x() +
                          ratio * (currSegment.getEnd().x() - currSegment.getStart().x());
            double newY = currSegment.getStart().y() +
                          ratio * (currSegment.getEnd().y() - currSegment.getStart().y());
            return Point{newX, newY};
        }

        // Subtract the length of the current segment from the total distance
        travelDistance -= segmentLength;

        // Update the segment index
        segmentIndex = (segmentIndex + 1) % polygonSegments.size();
    }

    return start;
}
