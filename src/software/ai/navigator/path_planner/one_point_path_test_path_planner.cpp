#include "software/ai/navigator/path_planner/one_point_path_test_path_planner.h"

/**
 * This file contains the implementation of a one point path test path planner
 * which returns a path consisting of one point if start == destination
 * Note that this is an invalid path, so the navigator won't like it
 */

std::optional<Path> OnePointPathTestPathPlanner::findPath(
    const Point &start, const Point &destination, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &obstacles)
{
    if (start == destination)
    {
        return Path(std::vector<Point>({start}));
    }
    else
    {
        return std::nullopt;
    }
}
