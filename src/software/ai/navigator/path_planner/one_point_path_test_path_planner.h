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
     * @param field  WARNING: OnePointPathTestPathPlanner does not respect the field
     * @param obstacles WARNING: OnePointPathTestPathPlanner does not avoid obstacles
     *
     * @return a vector that has one point
     */
    PathType findPath(const Point &start, const Point &destination, const Field &field,
                      const std::vector<Obstacle> &obstacles) override;
};
