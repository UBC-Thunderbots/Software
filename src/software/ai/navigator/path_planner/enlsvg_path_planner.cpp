#include "enlsvg_path_planner.h"

EnlsvgPathPlanner::EnlsvgPathPlanner(
    const Rectangle &navigable_area, const std::vector<ObstaclePtr> &obstacles)
{
    num_grid_rows = static_cast<int>(round(navigable_area.xLength() / (double) SIZE_OF_GRID_CELL_IN_METERS));
    num_grid_cols = static_cast<int>(round(navigable_area.yLength() / (double) SIZE_OF_GRID_CELL_IN_METERS));
    
    grid = std::make_unique<Pathfinding::Grid>(num_grid_rows, num_grid_cols);
    createObstaclesInGrid(obstacles);
    algo = std::make_unique<const Pathfinding::ENLSVG::Algorithm>(*grid);
    mem = std::make_unique<Pathfinding::ENLSVG::Memory>(*algo);

}

// TODO: check whether this const reference thing works because it shouldn't here
void EnlsvgPathPlanner::createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles) const
{
    for (auto &obstacle : obstacles)
    {
        auto blocked_points = obstacle->rasterize(SIZE_OF_GRID_CELL_IN_METERS);
        
        for (Point &blocked_point : blocked_points)
        {
            EnlsvgPathPlanner::GridVertex blocked_coord = convertPointToGridVertex(blocked_point);
            if (isCoordNavigable(blocked_coord))
            {
                grid->setBlocked(blocked_coord.x, blocked_coord.y, true);
            }
        }
    }
}

bool EnlsvgPathPlanner::isCoordNavigable(const GridVertex& gv) const
{
    //TODO
    return false;
}

std::optional<Path> EnlsvgPathPlanner::findPath(
    const Point &start, const Point &end, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &obstacles)
{
    
    EnlsvgPathPlanner::GridVertex s = convertPointToGridVertex(start);
    EnlsvgPathPlanner::GridVertex e = convertPointToGridVertex(end);
    EnlsvgPathPlanner::GridPath path = algo->computePath(*mem, s.x, s.y, e.x, e.y);
    return convertGridPathToPath(path);
}

Pathfinding::GridVertex EnlsvgPathPlanner::convertPointToGridVertex(
    const Point &p) const
{
    return Pathfinding::GridVertex(
        static_cast<int>(round(p.x() / (double) SIZE_OF_GRID_CELL_IN_METERS)),
        static_cast<int>(round(p.y() / (double) SIZE_OF_GRID_CELL_IN_METERS)));
}

Point EnlsvgPathPlanner::convertGridVertexToPoint(
    const Pathfinding::GridVertex &c) const
{
    return Point(static_cast<double>(c.x * SIZE_OF_GRID_CELL_IN_METERS), static_cast<double>(c.y / SIZE_OF_GRID_CELL_IN_METERS));
}

std::optional<Path> EnlsvgPathPlanner::convertGridPathToPath(
    const EnlsvgPathPlanner::GridPath& p) const
{
    if (!p.size())
    {
        return std::nullopt;
    }
    
    std::vector<Point> path;
    for (auto gv : p)
    {
        path.emplace_back(convertGridVertexToPoint(gv));
    }
    
    return std::optional(Path(path));
}

