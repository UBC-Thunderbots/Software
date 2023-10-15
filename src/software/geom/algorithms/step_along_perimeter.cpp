#include <vector>
#include <numeric>
#include "software/geom/algorithms/step_along_perimeter.h"
#include "software/geom/algorithms/collinear.h"
#include "software/geom/algorithms/distance.h"
#include "software/geom/segment.h"


Point stepAlongPerimeter(const Polygon& polygon, const Point& start, double travelDistance) {
    if (travelDistance == 0.0) {
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

    bool isClockwise = travelDistance > 0;

    // perimeter, can add this to the polygon class
    double perimeter = std::accumulate(polygonSegments.begin(), polygonSegments.end(), 0.0,
                                      [](double acc, const Segment& seg) {
                                          return acc + seg.length();
                                      }
    );

    // if travel distance is negative, it can be equal to perimeter - |travelDistance|.
    // the fmod function is to support wrapping around and negative distance
    travelDistance = isClockwise ? travelDistance : perimeter - std::fmod(std::abs(travelDistance), perimeter);
    while (travelDistance > 0) {
        Segment currSegment = polygonSegments[segmentIdx];

        double segmentLength = currSegment == polygonSegments[startSegmentIdx] ? distance(start, currSegment.getEnd()) : currSegment.length();

        // If the remaining distance to travel is less than or equal to the length
        // of the current segment, calculate the final point and return it
        if (travelDistance <= segmentLength) {
            double ratio = travelDistance / segmentLength;
            double newX =
                    currSegment.getStart().x() + ratio * (currSegment.getEnd().x() - currSegment.getStart().x());
            double newY =
                    currSegment.getStart().y() + ratio * (currSegment.getEnd().y() - currSegment.getStart().y());
            return Point{newX, newY};
        }

        // Subtract the length of the current segment from the total distance
        travelDistance -= segmentLength;

        // Update the segment index
        segmentIdx = (segmentIdx + 1) % polygonSegments.size();
    }

    return start;

}  

