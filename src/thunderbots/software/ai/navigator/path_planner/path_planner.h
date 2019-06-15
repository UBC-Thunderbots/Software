#pragma once
#include <optional>
#include <vector>

#include "ai/navigator/obstacle/obstacle.h"
#include "geom/point.h"

/**
 * PathPlanner is an interface for a path planner that,
 * given start and destination points, a list of obstacles,
 * and a violation function, will return the 'best' path
 * from start to the destination if a path exists, otherwise
 * it will return nothing.
 */

class PathPlanner
{
   public:
    /**
     * Typedef for a violation function, which consumes a Point
     * and returns the distance that that point is violating a
     * boundary by, in metres.
     */
    using ViolationFunction = std::function<double(const Point &)>;

    /**
     * Returns a path from start to dest given obstacles and a
     * violation, otherwise returns std::nullopt if a path is
     * not found.
     *
     * @param start start point
     * @param dest destination point
     * @param obstacles a vector of obstacles
     * @param violation_function a function that returns the distance
     *                          that a point is violating a boundary by
     * @return
     */
    virtual std::optional<std::vector<Point>> findPath(
        const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
        const ViolationFunction &violation_function) = 0;

    virtual std::optional<std::vector<Point>> findPath(const Point &start,
                                                       const Point &dest) = 0;

    virtual ~PathPlanner() = default;
};
