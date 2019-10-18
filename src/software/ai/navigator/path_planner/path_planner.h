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
     * Returns a path between start and destination.
     * (consider the case where we have a "StraightLinePathPlanner" for testing purposes)
     *
     * @param start start point
     * @param destination destination point
     * @param field field
     * @param obstacles obstacles to avoid
     *
     * @return a path between start and destination
     * 		if no valid path then return an empty vector
     * 		if valid path then return either
     * 		a vector of points or a vector of curves
     * 		    * The vector of points must include the start point and end point
     */
    virtual PathType findPath(const Point &start, const Point &destination,
                              const Field &field,
                              const std::vector<Obstacle> &obstacles) = 0;

    virtual ~PathPlanner() = default;
};
