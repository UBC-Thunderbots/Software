#include "global_path_planner.h"

GlobalPathPlanner::GlobalPathPlanner(const Rectangle &navigable_area, const World &world)
{
    num_grid_rows = static_cast<int>(navigable_area.xLength() / RESOLUTION_IN_CM);
    num_grid_cols = static_cast<int>(navigable_area.yLength() / RESOLUTION_IN_CM);
    
    grid(num_grid_rows, num_grid_cols);
    
    algo(grid);
    mem(algo);
}

std::optional<Path> GlobalPathPlanner::findPath(
    const Point &start, const Point &end, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &obstacles)
{
    return std::nullopt;
}