#pragma once
#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * NoPathTestPathPlanner is a very trivial implementation
 * of the PathPlanner interface.
 */

class NoPathTestPathPlanner : public PathPlanner
{
   public:
    /**
     * Returns an empty path
     *
     * @param start start point
     * @param destination destination point
     * @param navigableArea WARNING: NoPathTestPathPlanner does not respect the
     * navigableArea
     * @param obstacles WARNING: NoPathTestPathPlanner does not avoid obstacles
     *
     * @return a vector that is {}
     */
    Path findPath(const Point &start, const Point &destination,
                  const Rectangle &navigableArea,
                  const std::vector<Obstacle> &obstacles) override;
};
