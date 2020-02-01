#include "software/new_geom/segment.h"
#include "software/geom/util.h"

/**
 * Computes the intersection of two segments.
 *
 * @param first
 *
 * @param second
 *
 * @return  one of:
 *          - an empty vector if no intersections
 *          - a vector containing the point of intersection of a single point intersection
 *          - a vector containing two points representing the line segment of the overlap if both segments are collinear
 *          and overlapping
 */
std::vector<Point> intersection(const Segment &first, const Segment &second);
