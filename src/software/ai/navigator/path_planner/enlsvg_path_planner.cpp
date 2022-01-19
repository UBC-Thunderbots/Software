#include "enlsvg_path_planner.h"

EnlsvgPathPlanner::EnlsvgPathPlanner(
    const Rectangle &navigable_area, const std::vector<ObstaclePtr> &obstacles, double grid_boundary_offset, 
    double resolution)
    :   resolution(resolution),
        num_grid_rows(static_cast<int>(round(navigable_area.xLength() / resolution))),
        num_grid_cols(static_cast<int>(round(navigable_area.yLength() / resolution))),
        origin(navigable_area.negXNegYCorner()) ,
        max_navigable_y_enlsvg_point(convertPointToEnlsvgPoint(navigable_area.posXPosYCorner()).y),
        max_navigable_x_enlsvg_point(convertPointToEnlsvgPoint(navigable_area.posXPosYCorner()).x),
        grid(std::make_unique<EnlsvgGrid>(num_grid_rows, num_grid_cols))
{
    createObstaclesInGrid(obstacles, grid_boundary_offset);
    algo = std::make_unique<const EnlsvgAlgorithm>(*grid);
    mem = std::make_unique<EnlsvgMemory>(*algo);
}

void EnlsvgPathPlanner::createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles, double boundary_margin) const
{
    double offset_in_enlsvg = boundary_margin / resolution;
    for (int x = 0; x < offset_in_enlsvg; ++x)
    {
        for (int y = 0; y < num_grid_cols; ++y)
        {
            grid->setBlocked(x, y, true);
            grid->setBlocked(num_grid_rows-1-x, y, true);
        }
    }
    for (int y = 0; y < offset_in_enlsvg; ++y)
    {
        for (int x = 0; x < num_grid_rows; ++x)
        {
            grid->setBlocked(x, y, true);
            grid->setBlocked(x, num_grid_cols-1-y, true);
        }
    }
    
    for (auto &obstacle : obstacles)
    {
        auto blocked_points = obstacle->rasterize(resolution);
        
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
    return (ep.x >= 0 && ep.x < max_navigable_x_enlsvg_point)
        && (ep.y >= 0 && ep.y < max_navigable_y_enlsvg_point);
}

std::optional<Path> EnlsvgPathPlanner::findPath(
    const Point &start, const Point &end, const Rectangle &navigable_area,
    const std::vector<ObstaclePtr> &ignored)
{
    // Check if start and end coordinates are in navigable area and return null if it isn't
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
    
    // Find closest unblocked points in case the start and end positions are inside obstacles
    EnlsvgPoint s   = convertPointToEnlsvgPoint(start);
    EnlsvgPoint e   = convertPointToEnlsvgPoint(end);
    auto new_start  = findClosestUnblockedEnlsvgPoint(s);
    auto new_end    = findClosestUnblockedEnlsvgPoint(e);
    
    if (new_start == std::nullopt || new_end == std::nullopt)
    {
        return std::nullopt;
    }
    
    EnlsvgPath enlsvgPath = algo->computePath(*mem, new_start.value().x, new_start.value().y,
                                              new_end.value().x, new_end.value().y);
    std::optional<Path> path = convertEnlsvgPathToPath(enlsvgPath);
    if (path == std::nullopt)
    {
        return std::nullopt;
    }
    
    std::vector<Point> path_points = path.value().getKnots();
    
    // If start was initially blocked, add the start point
    if (new_start.value() != s)
    {
        path_points.insert(path_points.begin(), start);
    }
    
    // If the end point wasn't blocked, then replace the end with the actual end because some details get lose due to 
    // the grid resolution
    if (new_end.value() == e)
    {
        path_points.pop_back();
        path_points.emplace_back(end);
    }
     
    // Make sure start point corresponds exactly with the given start
    path_points.erase(path_points.begin());
    path_points.insert(path_points.begin(), start);
    
    // Due to processing, it is possible that the first two points may be very close together, this will fix that
    if (path_points.size() > 2
       && (path_points[0] - path_points[1]).length() < resolution)
    {
        path_points.erase(path_points.begin() + 1);
    }
    
    return Path(path_points);
}

EnlsvgPathPlanner::EnlsvgPoint EnlsvgPathPlanner::convertPointToEnlsvgPoint(
    const Point &p) const
{
    return EnlsvgPoint(
        static_cast<int>(round((p.x() - origin.x())/ (double) resolution)),
        static_cast<int>(round((p.y() - origin.y()) / (double) resolution)));
}

Point EnlsvgPathPlanner::convertEnlsvgPointToPoint(
    const EnlsvgPoint &c) const
{
    return Point(
        static_cast<double>(c.x * resolution + origin.x()), 
        static_cast<double>(c.y * resolution + origin.y()));
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
    // Uses DFS to find the closest unblocked cell by looking at nearby cells
    std::queue<EnlsvgPoint> q;
    std::vector<EnlsvgPoint> visited;
    q.push(ep);
    visited.emplace_back(ep);
    while (!q.empty())
    {
        EnlsvgPoint test_coord = q.front();
        q.pop();
        if (!isBlocked(test_coord))
        {
            return std::optional<EnlsvgPoint>(test_coord);
        }
        for (int i = 1; i > -2; i -= 2)
        {
            EnlsvgPoint next_coord = EnlsvgPoint(test_coord.x+i, test_coord.y);
            for (int j = -1; j < 2; j += 2)
            {
                if (isCoordNavigable(next_coord) 
                    && (std::find(visited.begin(), visited.end(), next_coord) == visited.end()))
                {
                    q.push(next_coord);
                    visited.emplace_back(next_coord);
                }
                next_coord = EnlsvgPoint(test_coord.x, test_coord.y+i);
            }
        }
    }
    return std::nullopt;
}

bool EnlsvgPathPlanner::isBlocked(const EnlsvgPoint &ep) const
{
    return grid->isBlocked(ep.x, ep.y);
}
