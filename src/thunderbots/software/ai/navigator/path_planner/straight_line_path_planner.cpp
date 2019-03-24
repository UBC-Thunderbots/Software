#include "ai/navigator/path_planner/straight_line_path_planner.h"

std::optional<std::vector<Point>> StraightLinePathPlanner::findPath(
    const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
    const ViolationFunction &violation_function)
{
    return std::make_optional<std::vector<Point>>({start, dest});
}
