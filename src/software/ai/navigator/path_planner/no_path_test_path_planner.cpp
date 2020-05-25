#include "software/ai/navigator/path_planner/no_path_test_path_planner.h"

/**
 * This file contains the implementation of a no path test path planner
 * which returns a path consisting of no points.
 */

std::optional<Path> NoPathTestPathPlanner::findPath(
    const Point &start, const Point &destination, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &obstacles)
{
    return std::nullopt;
}
