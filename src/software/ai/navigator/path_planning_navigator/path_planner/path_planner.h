#pragma once
#include <optional>
#include <vector>

#include "software/ai/navigator/path_planning_navigator/obstacle/obstacle.h"
#include "software/geom/point.h"

/**
 * PathPlanner is an interface for a path planner that,
 * given start and destination points, a list of obstacles,
 * will return the 'best' path from start to the destination
 * if a path exists, otherwise it will return nothing.
 */

class PathPlanner
{
   public:
    virtual std::optional<std::vector<Point>> findPath(const Point &start,
                                                       const Point &dest) = 0;

    virtual ~PathPlanner() = default;
};
