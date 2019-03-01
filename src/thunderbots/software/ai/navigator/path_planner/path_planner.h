#pragma once
#include <ai/world/world.h>

#include <optional>
#include <vector>

#include "geom/point.h"

typedef std::function<const double(const Point &)> ViolationFunction;

class PathPlanner
{
   public:
    /**
     * Finds a path from start to dest if it exists, otherwise return std::nullopt.
     * @param violation_function A function that defines how much a Point intrudes
     *                           on the boundaries of an obstacle.
     * @param start the start point.
     * @param dest the destination point.
     * @return a path from start to dest if it exists, otherwise std::nullopt.
     */
    virtual std::optional<std::vector<Point>> findPath(
        const ViolationFunction &violation_function, const Point &start,
        const Point &dest) = 0;
    virtual ~PathPlanner() = default;
};
