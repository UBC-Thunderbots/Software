#include "global_path_planner.h"

GlobalPathPlanner::GlobalPathPlanner(const Rectangle &navigable_area, const World &world)
{
    num_grid_rows = static_cast<int>(round(navigable_area.xLength() / (double) RESOLUTION_IN_CM));
    num_grid_cols = static_cast<int>(round(navigable_area.yLength() / (double) RESOLUTION_IN_CM));
    
    grid = std::make_unique<const Pathfinding::Grid>(num_grid_rows, num_grid_cols);
    algo = std::make_unique<const Pathfinding::ENLSVG::Algorithm>(*grid);
    mem = std::make_unique<Pathfinding::ENLSVG::Memory>(*algo);
}

std::optional<Path> GlobalPathPlanner::findPath(
    const Point &start, const Point &end, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &obstacles)
{
    
    GlobalPathPlanner::Coordinate s = convertPointToCoord(start);
    GlobalPathPlanner::Coordinate e = convertPointToCoord(end);
    Pathfinding::Path path = algo->computePath(*mem, s.x, s.y, e.x, e.y);
    return std::nullopt;
}

GlobalPathPlanner::Coordinate GlobalPathPlanner::convertPointToCoord(
    const Point &p) const
{
    return Coordinate(static_cast<int>(round(p.x() / (double) RESOLUTION_IN_CM)),
                      static_cast<int>(round(p.y() / (double) RESOLUTION_IN_CM)));
}

Point GlobalPathPlanner::convertCoordToPoint(
    const Coordinate &c) const
{
    return Point(static_cast<double>(c.x * RESOLUTION_IN_CM), static_cast<double>(c.y / RESOLUTION_IN_CM));
}