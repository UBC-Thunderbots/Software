//
// Created by jordan on 3/23/19.
//

#include "straight_line_path_planner.h"

std::optional<std::vector<Point>> StraightLinePathPlanner::findPath(
    const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
    const ViolationFunction &violation_function)
{
    return std::make_optional<std::vector<Point>>({start, dest});
}
