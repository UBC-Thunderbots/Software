#pragma once
#include <ai/world/world.h>

#include <optional>
#include <vector>

#include "geom/point.h"

class PathPlanner
{
   public:
    /**
     * Finds a path from start to dest if it exists, otherwise return std::nullopt.
     * @param start the start point.
     * @param dest the destination point.
     * @return a path from start to dest if it exists, otherwise std::nullopt.
     */
    virtual std::optional<std::vector<Point>> findPath(const Point &start,
                                                       const Point &dest) = 0;
    virtual ~PathPlanner()                                                = default;
};
