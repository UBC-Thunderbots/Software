#include "software/ai/navigator/path_planner/one_point_path_test_path_planner.h"

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
