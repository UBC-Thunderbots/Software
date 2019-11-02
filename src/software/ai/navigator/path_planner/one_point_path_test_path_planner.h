#pragma once
#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * OnePointPathTestPathPlanner is a very trivial implementation
 * of the PathPlanner interface.
 */

class OnePointPathTestPathPlanner : public PathPlanner
{
   public:
    /**
     * Returns an one point path
     *
     * @param start start point
     * @param destination destination point
     * @param navigable_area WARNING: OnePointPathTestPathPlanner does not respect the
     * navigable_area
     * @param obstacles WARNING: OnePointPathTestPathPlanner does not avoid obstacles
     *
     * @return a vector that has one point
     */
    Path findPath(const Point &start, const Point &destination,
                  const Rectangle &navigable_area,
                  const std::vector<Obstacle> &obstacles) override;
};
