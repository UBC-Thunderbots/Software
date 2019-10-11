#pragma once
#include <variant>
#include <vector>

#include "software/ai/navigator/curve/curve.h"
#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/geom/point.h"
#include "software/world/field.h"

/**
 * PathPlanner is an interface for a path planner that,
 * given start and destination points, a list of obstacles,
 * will return the 'best' path from start to the destination
 * if a path exists, otherwise it will return nothing.
 */

using PathType = std::variant<std::vector<Curve>, std::vector<Point>>;

class PathPlanner
{
   public:
    /**
     * Returns a path between start and dest.
     * (consider the case where we have a "StraightLinePathPlanner" for testing purposes)
     *
     * @param start start point
     * @param dest destination point
     * @param field field
     * @param obstacles obstacles to avoid
     *
     * @return a path between start and dest
     * 		if no valid path then return an empty vector
     * 		if spline navigator then return a std::vector of Curve
     * 		if point navigator then return a std::vector of points
     * 		    * this vector must include the start point and end point
     */
    virtual PathType findPath(const Point &start, const Point &dest, const Field &field,
                              const std::vector<Obstacle> &obstacles) = 0;

    virtual ~PathPlanner() = default;
};
