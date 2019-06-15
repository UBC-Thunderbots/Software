#pragma once
#include "ai/navigator/path_planner/path_planner.h"

/**
 * StraightLinePathPlanner is a very trivial implementation
 * of the PathPlanner interface.
 */

class StraightLinePathPlanner : public PathPlanner
{
   public:
    /**
     * Returns a path that is a straight line between start and dest.
     * @param start start point
     * @param dest destination point
     * @param obstacles unused parameter
     * @param violation_function unused parameter
     * @return a vector that is {start, dest}
     */
    std::optional<std::vector<Point>> findPath(
        const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
        const ViolationFunction &violation_function) override;

    std::optional<std::vector<Point>> findPath(const Point &start,
                                               const Point &dest) override;
};
