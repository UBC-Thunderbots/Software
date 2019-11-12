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
     * @param field  WARNING: StraightLinePathPlanner does not respect the field
     * @param obstacles WARNING: StraightLinePathPlanner does not avoid obstacles
     *
     * @return a vector that is {start, destination}
     */
    PathType findPath(const Point &start, const Point &destination, const Field &field,
                      const std::vector<Obstacle> &obstacles) override;
};
