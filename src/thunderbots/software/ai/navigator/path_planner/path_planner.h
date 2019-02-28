#pragma once
#include <ai/world/world.h>

#include <optional>
#include <vector>

#include "geom/point.h"

typedef std::function<double(const Point &)> ViolationFunction;

class PathPlanner
{
   public:
    // TODO: consider passing a lambda to the path planner instead of the world
    virtual std::optional<std::vector<Point>> findPath(
        const ViolationFunction &violation_function, const Point &start,
        const Point &dest) = 0;
    virtual ~PathPlanner() = default;
};
