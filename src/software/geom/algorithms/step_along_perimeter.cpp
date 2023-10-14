#include <vector>
#include "software/geom/algorithms/step_along_perimeter.h"
#include "software/geom/algorithms/collinear.h"
//#include "software/geom/algorithms/distance.h"
#include "software/geom/segment.h"

Point stepAlongPerimeter(const Polygon& polygon, const Point& start, double distance) {
    if (distance == 0.0) {
        return start;
    }

    std::vector<Segment> polygonSegments = polygon.getSegments();
    int startSegmentIdx = 0;
    int i = 0;

    for (Segment& segment: polygonSegments) {

        Point segmentStart = segment.getStart();
        Point segmentEnd = segment.getEnd();

        if (collinear(segmentStart, start, segmentEnd)) {
            startSegmentIdx = i;
            break;
        }
        i++;
    }

    int segmentIdx = startSegmentIdx;

    bool isClockwise = distance > 0;
    distance = std::abs(distance);
    while (distance > 0) {
        Segment currSegment = polygonSegments[segmentIdx];
        double segmentLength = currSegment.length();

        if (distance <= segmentLength) {
            double ratio = distance / segmentLength;
            double newX =
                    currSegment.getStart().x() + ratio * (currSegment.getEnd().x() - currSegment.getStart().x());
            double newY =
                    currSegment.getStart().y() + ratio * (currSegment.getEnd().y() - currSegment.getStart().y());
            return Point{newX, newY};
        }

        distance -= segmentLength;
        if (isClockwise) {
            segmentIdx = (segmentIdx + 1) % polygonSegments.size();
        } else {
            segmentIdx = (segmentIdx == 0) ? polygonSegments.size() - 1 : segmentIdx - 1;
        }

    }

    return start;

}  

