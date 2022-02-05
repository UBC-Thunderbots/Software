#include "global_path_planner.h"

GlobalPathPlanner::GlobalPathPlanner(const World &world) { }

std::optional<Path> GlobalPathPlanner::findPath(const Point &start, const Point &end, 
                                                const std::set<MotionConstraint> &motion_constaints)
{
    return std::nullopt;
}