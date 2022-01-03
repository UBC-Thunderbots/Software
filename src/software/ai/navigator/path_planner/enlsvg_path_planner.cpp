#include "enlsvg_path_planner.h"

EnlsvgPathPlanner::EnlsvgPathPlanner(
    const Rectangle &navigable_area, const std::vector<ObstaclePtr> &obstacles)
    :   num_grid_rows(static_cast<int>(round(navigable_area.xLength() / (double) SIZE_OF_GRID_CELL_IN_METERS))),
        num_grid_cols(static_cast<int>(round(navigable_area.yLength() / (double) SIZE_OF_GRID_CELL_IN_METERS))),
        min_navigable_y_enlsvg_point(convertPointToEnlsvgPoint(navigable_area.negXNegYCorner()).y),
        min_navigable_x_enlsvg_point(convertPointToEnlsvgPoint(navigable_area.negXNegYCorner()).x),
        max_navigable_y_enlsvg_point(convertPointToEnlsvgPoint(navigable_area.posXPosYCorner()).y),
        max_navigable_x_enlsvg_point(convertPointToEnlsvgPoint(navigable_area.posXPosYCorner()).x),
        grid(std::make_unique<EnlsvgGrid>(num_grid_rows, num_grid_cols))
{
    createObstaclesInGrid(obstacles);
    algo = std::make_unique<const EnlsvgAlgorithm>(*grid);
    mem = std::make_unique<EnlsvgMemory>(*algo);

}

// TODO: check whether this const reference thing works because it shouldn't here
void EnlsvgPathPlanner::createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles) const
{
    for (auto &obstacle : obstacles)
    {
        auto blocked_points = obstacle->rasterize(SIZE_OF_GRID_CELL_IN_METERS);
        
        for (Point &blocked_point : blocked_points)
        {
            EnlsvgPoint blocked_coord = convertPointToEnlsvgPoint(blocked_point);
            if (isCoordNavigable(blocked_coord))
            {
                grid->setBlocked(blocked_coord.x, blocked_coord.y, true);
            }
        }
    }
}

bool EnlsvgPathPlanner::isCoordNavigable(const EnlsvgPoint& ep) const
{
    return (min_navigable_x_enlsvg_point >= ep.x && ep.x <= max_navigable_x_enlsvg_point)
        && (min_navigable_y_enlsvg_point >= ep.y && ep.y <= max_navigable_y_enlsvg_point);
}

std::optional<Path> EnlsvgPathPlanner::findPath(
    const Point &start, const Point &end, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &obstacles)
{
    bool navigable_area_contains_start =
        (start.x() >= navigable_area.xMin()) && (start.x() <= navigable_area.xMax()) &&
        (start.y() >= navigable_area.yMin()) && (start.y() <= navigable_area.yMax());
    bool navigable_area_contains_end =
        (end.x() >= navigable_area.xMin()) && (end.x() <= navigable_area.xMax()) &&
        (end.y() >= navigable_area.yMin()) && (end.y() <= navigable_area.yMax());
    if (!navigable_area_contains_start || !navigable_area_contains_end)
    {
        return std::nullopt;
    }
    
    EnlsvgPoint s = convertPointToEnlsvgPoint(start);
    EnlsvgPoint e = convertPointToEnlsvgPoint(end);
    EnlsvgPath path = algo->computePath(*mem, s.x, s.y, e.x, e.y);
    return convertEnlsvgPathToPath(path);
}

EnlsvgPathPlanner::EnlsvgPoint EnlsvgPathPlanner::convertPointToEnlsvgPoint(
    const Point &p) const
{
    return EnlsvgPoint(
        static_cast<int>(round(p.x() / (double) SIZE_OF_GRID_CELL_IN_METERS)),
        static_cast<int>(round(p.y() / (double) SIZE_OF_GRID_CELL_IN_METERS)));
}

Point EnlsvgPathPlanner::convertEnlsvgPointToPoint(
    const EnlsvgPoint &c) const
{
    return Point(static_cast<double>(c.x * SIZE_OF_GRID_CELL_IN_METERS), static_cast<double>(c.y / SIZE_OF_GRID_CELL_IN_METERS));
}

std::optional<Path> EnlsvgPathPlanner::convertEnlsvgPathToPath(
    const EnlsvgPath& p) const
{
    if (!p.size())
    {
        return std::nullopt;
    }
    
    std::vector<Point> path;
    for (auto gv : p)
    {
        path.emplace_back(convertEnlsvgPointToPoint(gv));
    }
    
    return std::optional(Path(path));
}

std::optional<EnlsvgPathPlanner::EnlsvgPoint> EnlsvgPathPlanner::findClosestUnblockedEnlsvgPoint(
    const EnlsvgPoint &ep) const
{
    std::queue<EnlsvgPoint> q;
    std::vector<EnlsvgPoint> visited;
    q.push(ep);
    while (!q.empty())
    {
        EnlsvgPoint test_coord = q.front();
        visited.emplace_back(test_coord);
        q.pop();
        if (!isBlocked(test_coord))
        {
            return std::optional<EnlsvgPoint>(test_coord);
        }
        for (int i = -1; i < 2; ++i)
        {
            for (int j = -1; j < 2; ++j)
            {
                EnlsvgPoint next_coord = EnlsvgPoint(ep.x+i, ep.y+j);
                if (isCoordNavigable(next_coord) 
                    && (std::find(visited.begin(), visited.end(), next_coord) == visited.end()))
                {
                    q.push(next_coord);
                }
            }
        }
    }
    return std::nullopt;
}

bool EnlsvgPathPlanner::isBlocked(const EnlsvgPoint &ep) const
{
    return grid->isBlocked(ep.x, ep.y);
}
