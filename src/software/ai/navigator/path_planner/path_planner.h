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

class PathPlanner
{
   public:
    /**
     * Returns a "good" path between start and dest.
     *
     * @param start start point
     * @param dest destination point
     * @param field field
     * @param obstacles obstacles to avoid
     *
     * @return the optimal path between start and dest
     * 		if no valid path then return std::monostate
     * 		if spline navigator then return a Curve
     * 		if point navigator then return a std::vector of points
     */
    virtual std::variant<std::monostate, Curve, std::vector<Point>> findPath(
        const Point &start, const Point &dest, const Field &field,
        const std::vector<Obstacle> &obstacles) = 0;

    virtual ~PathPlanner() = default;
};
