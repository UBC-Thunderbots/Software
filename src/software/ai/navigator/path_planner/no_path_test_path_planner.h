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
     * @param field  WARNING: NoPathTestPathPlanner does not respect the field
     * @param obstacles WARNING: NoPathTestPathPlanner does not avoid obstacles
     *
     * @return a vector that is {}
     */
    PathType findPath(const Point &start, const Point &destination, const Field &field,
                      const std::vector<Obstacle> &obstacles) override;
};
