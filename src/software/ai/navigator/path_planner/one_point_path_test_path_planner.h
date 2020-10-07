#pragma once

#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * OnePointPathTestPathPlanner returns a path consisting of one point
 * if start == destination
 * Otherwise, it returns no path
 * Note that this is an invalid path, so the navigator won't like it
 */
class OnePointPathTestPathPlanner : public PathPlanner
{
   public:
    std::optional<Path> findPath(const Point &start, const Point &destination,
                                 const Rectangle &navigable_area,
                                 const std::vector<ObstaclePtr> &obstacles) override;
};
