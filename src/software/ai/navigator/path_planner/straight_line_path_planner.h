#pragma once
#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * StraightLinePathPlanner is a very trivial implementation
 * of the PathPlanner interface.
 */

class StraightLinePathPlanner : public PathPlanner
{
   public:
    /**
     * Returns a path that is a straight line between start and destination.
     *
     * @param start start point
     * @param destination destination point
     * @param navigable_area WARNING: StraightLinePathPlanner does not respect the
     * navigable_area
     * @param obstacles WARNING: StraightLinePathPlanner does not avoid obstacles
     *
     * @return a vector that is {start, destination}
     */
    std::optional<Path> findPath(const Point &start, const Point &destination,
                                 const Rectangle &navigable_area,
                                 const std::vector<ObstaclePtr> &obstacles) override;
};
