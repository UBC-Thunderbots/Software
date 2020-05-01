#pragma once
#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * OnePointPathTestPathPlanner returns a path consisting of one point
 * if start == destination
 * Otherwise, it returns no path
 */
class OnePointPathTestPathPlanner : public PathPlanner
{
   public:
    std::optional<Path> findPath(const Point &start, const Point &destination,
                                 const Rectangle &navigable_area,
                                 const std::vector<ObstaclePtr> &obstacles) override;
};
