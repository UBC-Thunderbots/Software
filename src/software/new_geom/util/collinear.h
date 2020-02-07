#include "software/new_geom/point.h"
#include "software/new_geom/segment.h"

/**
 * Returns true if the given points are collinear, false otherwise.
 *
 * @params a, b, c the given points
 *
 * @return true if the given points are collinear, false otherwise
 */
bool collinear(const Point &a, const Point &b, const Point &c);

/**
 * Returns true if the given segments are collinear, false otherwise.
 *
 * @params first, second the given segments
 *
 * @return true if the given segments are collinear, false otherwise
 */
bool collinear(const Segment &first, const Segment &second);
