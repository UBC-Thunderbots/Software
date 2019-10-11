#include "software/ai/navigator/path_planner/straight_line_path_planner.h"

/**
 * This file contains the implementation of a straight line path planner
 * which returns a path consisting of only the start and destination
 * points.
 */

std::variant<std::monostate, Curve, std::vector<Point>> StraightLinePathPlanner::findPath(
    const Point &start, const Point &dest, const Field &field,
    const std::vector<Obstacle> &obstacles)
{
    return std::variant<std::monostate, Curve, std::vector<Point>>(
        std::vector<Point>({start, dest}));
}
