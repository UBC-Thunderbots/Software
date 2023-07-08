#include "software/ai/navigator/path_planner/enlsvg_path_planner.h"

EnlsvgPathPlanner::EnlsvgPathPlanner(const Rectangle &navigable_area,
                                     const std::vector<ObstaclePtr> &obstacles,
                                     double grid_boundary_offset, double resolution)
    : resolution(resolution),
      num_grid_rows(
          static_cast<unsigned int>(round(navigable_area.xLength() / resolution))),
      num_grid_cols(
          static_cast<unsigned int>(round(navigable_area.yLength() / resolution))),
      origin(navigable_area.negXNegYCorner()),
      max_navigable_y_enlsvg_point(
          convertPointToEnlsvgPoint(navigable_area.posXPosYCorner()).y),
      max_navigable_x_enlsvg_point(
          convertPointToEnlsvgPoint(navigable_area.posXPosYCorner()).x),
      enlsvg_grid(std::make_unique<EnlsvgGrid>(num_grid_rows, num_grid_cols))
{
    createObstaclesInGrid(obstacles, grid_boundary_offset);
    enlsvg_algo = std::make_unique<const EnlsvgAlgorithm>(*enlsvg_grid);
    enlsvg_mem  = std::make_unique<EnlsvgMemory>(*enlsvg_algo);
}

void EnlsvgPathPlanner::createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles,
                                              double boundary_margin) const
{
    // block boundary areas
    double offset_in_enlsvg = boundary_margin / resolution;
    for (unsigned x = 0; x < offset_in_enlsvg; ++x)
    {
        for (unsigned y = 0; y < num_grid_cols; ++y)
        {
            enlsvg_grid->setBlocked(x, y, true);
            enlsvg_grid->setBlocked(num_grid_rows - 1 - x, y, true);
        }
    }
    for (unsigned y = 0; y < offset_in_enlsvg; ++y)
    {
        for (unsigned x = 0; x < num_grid_rows; ++x)
        {
            enlsvg_grid->setBlocked(x, y, true);
            enlsvg_grid->setBlocked(x, num_grid_cols - 1 - y, true);
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
                enlsvg_grid->setBlocked(blocked_coord.x, blocked_coord.y, true);
            }
        }
    }
}

bool EnlsvgPathPlanner::isCoordNavigable(const EnlsvgPoint &ep) const
{
    return (ep.x >= 0 && ep.x < max_navigable_x_enlsvg_point) &&
           (ep.y >= 0 && ep.y < max_navigable_y_enlsvg_point);
}

std::optional<Path> EnlsvgPathPlanner::findPath(const Point &start,
                                                const Point &end) const
{
    EnlsvgPoint enlsvg_start = convertPointToEnlsvgPoint(start);
    EnlsvgPoint enlsvg_end   = convertPointToEnlsvgPoint(end);

    // closest unblocked points to requested start and end
    auto new_start = findClosestUnblockedEnlsvgPoint(enlsvg_start, enlsvg_end);
    auto new_end   = findClosestUnblockedEnlsvgPoint(enlsvg_end, enlsvg_start);

    if (new_start == std::nullopt || new_end == std::nullopt)
    {
        LOG(WARNING)
            << "Unable to find a path; Unable to find a nearby start and/or end point that isn't blocked "
            << "within the navigable area; no path found between " << start << " and "
            << end << std::endl;
        return std::nullopt;
    }

    // If the start and end points are very close together and are unblocked, just return
    // a straight line path
    if ((start != end) && (enlsvg_start == enlsvg_end) && (new_start == enlsvg_start) &&
        (new_end == enlsvg_end))
    {
        return Path({start, end});
    }

    // If the new unblocked points are equal, set the unblocked end point as the end point
    // of the path
    if (new_start == new_end)
    {
        return Path({start, convertEnlsvgPointToPoint(new_end.value())});
    }

    EnlsvgPath enlsvgPath =
        enlsvg_algo->computePath(*enlsvg_mem, new_start.value().x, new_start.value().y,
                                 new_end.value().x, new_end.value().y);
    std::optional<Path> path = convertEnlsvgPathToPath(enlsvgPath);
    if (path == std::nullopt)
    {
        LOG(INFO) << "The path planner was unable to find a path between " << start
                  << " and " << end << std::endl;
        return std::nullopt;
    }

    // If start was initially blocked, add the start point
    if (new_start.value() != enlsvg_start)
    {
        path.value().insert(path.value().begin(), start);
    }

    // If the end point wasn't blocked, then replace the end with the actual end because
    // some details get lost due to the grid resolution
    if (new_end.value() == enlsvg_end)
    {
        path.value().pop_back();
        path.value().emplace_back(end);
    }

    // Make sure start point corresponds exactly with the given start
    path.value().erase(path.value().begin());
    path.value().insert(path.value().begin(), start);

    // Due to processing, it is possible that the first two points may be very close
    // together, this will fix that
    if (path.value().size() > 2 &&
        (path.value()[0] - path.value()[1]).length() < resolution)
    {
        path.value().erase(path.value().begin() + 1);
    }

    return Path(path.value());
}

EnlsvgPathPlanner::EnlsvgPoint EnlsvgPathPlanner::convertPointToEnlsvgPoint(
    const Point &p) const
{
    return EnlsvgPoint(
        static_cast<int>(round((p.x() - origin.x()) / (double)resolution)),
        static_cast<int>(round((p.y() - origin.y()) / (double)resolution)));
}

Point EnlsvgPathPlanner::convertEnlsvgPointToPoint(const EnlsvgPoint &c) const
{
    return Point(static_cast<double>(c.x * resolution + origin.x()),
                 static_cast<double>(c.y * resolution + origin.y()));
}

std::optional<Path> EnlsvgPathPlanner::convertEnlsvgPathToPath(const EnlsvgPath &p) const
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

std::optional<EnlsvgPathPlanner::EnlsvgPoint>
EnlsvgPathPlanner::findClosestUnblockedEnlsvgPoint(const EnlsvgPoint &ep,
                                                   const EnlsvgPoint &other_point) const
{
    // Try to short circuit
    if (!isBlocked(ep))
    {
        return ep;
    }

    // Uses BFS to find the closest unblocked cell by looking at nearby cells
    std::queue<EnlsvgPoint> q;
    std::unordered_set<EnlsvgPoint, HashEnlsvgPoint> visited;
    q.push(ep);
    visited.emplace(ep);
    while (!q.empty())
    {
        EnlsvgPoint test_coord = q.front();
        q.pop();
        if (!isBlocked(test_coord))
        {
            return std::optional<EnlsvgPoint>(test_coord);
        }
        // Place immediately horizontal and vertical coordinates on the list of nodes to
        // check
        EnlsvgPoint next_coords[4] = {{test_coord.x + 1, test_coord.y},
                                      {test_coord.x, test_coord.y - 1},
                                      {test_coord.x, test_coord.y + 1},
                                      {test_coord.x - 1, test_coord.y}};

        // Order based on whichever point is closer to the other point we are coming
        // from/going to. This should help reduce the chance of the starting point moving
        // away from the end point (or vise versa), thus reducing the bug where robots get
        // stuck in the corner of the defense area moving back and forth due to the start
        // point being blocked.
        std::sort(std::begin(next_coords), std::end(next_coords),
                  [other_point](const EnlsvgPoint &a, const EnlsvgPoint &b) {
                      return std::hypot(a.x - other_point.x, a.y - other_point.y) <
                             std::hypot(b.x - other_point.x, b.y - other_point.y);
                  });

        for (auto &next_coord : next_coords)
        {
            if (isCoordNavigable(next_coord) && visited.count(next_coord) == 0)
            {
                q.push(next_coord);
                visited.emplace(next_coord);
            }
        }
    }
    return std::nullopt;
}

bool EnlsvgPathPlanner::isBlocked(const EnlsvgPoint &ep) const
{
    return !isCoordNavigable(ep) || enlsvg_grid->isBlocked(ep.x, ep.y);
}

double EnlsvgPathPlanner::pathLength(const std::vector<Point> &path_points,
                                     const Point &robot_position)
{
    double length = 0.0;
    Point prev_pt = robot_position;
    for (const auto &pt : path_points)
    {
        length += (pt - prev_pt).length();
        prev_pt = pt;
    }
    return length;
}
