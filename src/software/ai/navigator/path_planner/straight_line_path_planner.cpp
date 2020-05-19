#include "software/ai/navigator/path_planner/straight_line_path_planner.h"

/**
 * This file contains the implementation of a straight line path planner
 * which returns a path consisting of only the start and destination
 * points.
 */

std::optional<Path> StraightLinePathPlanner::findPath(
    const Point &start, const Point &destination, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &obstacles)
{
    return Path(std::vector<Point>({start, destination}));
}
