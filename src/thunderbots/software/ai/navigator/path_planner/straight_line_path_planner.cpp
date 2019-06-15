#include "ai/navigator/path_planner/straight_line_path_planner.h"

/**
 * This file contains the implementation of a straight line path planner
 * which returns a path consisting of only the start and destination
 * points.
 */

std::optional<std::vector<Point>> StraightLinePathPlanner::findPath(
    const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
    const ViolationFunction &violation_function)
{
    return std::make_optional<std::vector<Point>>({start, dest});
}

std::optional<std::vector<Point>> StraightLinePathPlanner::findPath(const Point &start,
                                                                    const Point &dest)
{
    return std::make_optional<std::vector<Point>>({start, dest});
}
