#include "software/ai/navigator/path_planner/one_point_path_test_path_planner.h"

/**
 * This file contains the implementation of a one point path test path planner
 * which returns a path consisting of one point
 * Note that this is an invalid path, so the navigator won't like it
 */

PathType OnePointPathTestPathPlanner::findPath(const Point &start,
                                               const Point &destination,
                                               const Field &field,
                                               const std::vector<Obstacle> &obstacles)
{
    return PathType(std::vector<Point>({Point(1.0, 1.0)}));
}
