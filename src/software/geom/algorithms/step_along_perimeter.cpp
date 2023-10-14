#include <vector>
#include "software/geom/algorithms/step_along_perimeter.h"
#include "software/geom/algorithms/collinear.h"
#include "software/geom/segment.h"

Point stepAlongPerimeter(const Polygon& polygon, const Point& start, double distance) {
    if (distance == 0.0) {
        return start;
    }

    std::vector<Segment> polygonSegments = polygon.getSegments();
    std::size_t startSegmentIdx = 0;

    int i = 0;

    // find initial segment which contains start point
    for (Segment& segment: polygonSegments) {

        Point segmentStart = segment.getStart();
        Point segmentEnd = segment.getEnd();

        if (collinear(segmentStart, start, segmentEnd)) {
            startSegmentIdx = i;
            break;
        }
        i++;
    }

    std::size_t segmentIdx = startSegmentIdx;

    bool isClockwise = distance > 0;

    // Always work with positive distance in the while loop
    distance = std::abs(distance);
    while (distance > 0) {
        Segment currSegment = polygonSegments[segmentIdx];
        double segmentLength = currSegment.length();

        // If the remaining distance to travel is less than or equal to the length
        // of the current segment, calculate the final point and return it
        if (distance <= segmentLength) {
            double ratio = distance / segmentLength;
            double newX =
                    currSegment.getStart().x() + ratio * (currSegment.getEnd().x() - currSegment.getStart().x());
            double newY =
                    currSegment.getStart().y() + ratio * (currSegment.getEnd().y() - currSegment.getStart().y());
            return Point{newX, newY};
        }

        // Subtract the length of the current segment from the total distance
        distance -= segmentLength;

        // Update the segment index based on the direction of traversal
        if (isClockwise) {
            segmentIdx = (segmentIdx + 1) % polygonSegments.size();
        } else {
            segmentIdx = (segmentIdx == 0) ? polygonSegments.size() - 1 : segmentIdx - 1;
        }

    }

    return start;

}  

