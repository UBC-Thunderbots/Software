#pragma once
#include <ai/world/world.h>

#include <optional>
#include <vector>

#include "geom/point.h"

class PathPlanner
{
   public:
    // TODO: consider passing a lambda to the path planner instead of the world
    virtual std::optional<std::vector<Point>> findPath(const World &world,
                                                       const Point &start,
                                                       const Point &dest) = 0;
    virtual ~PathPlanner()                                                = default;
};