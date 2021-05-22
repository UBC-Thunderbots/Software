#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

#include <stack>

#include "software/geom/algorithms/distance.h"
#include "software/geom/algorithms/intersects.h"
#include "software/logger/logger.h"

ThetaStarPathPlanner::ThetaStarPathPlanner()
    : num_grid_rows(0),
      num_grid_cols(0),
      max_navigable_x_coord(0),
      max_navigable_y_coord(0)
{
}

bool ThetaStarPathPlanner::isCoordNavigable(const Coordinate &coord) const
{
    // Returns true if row number and column number is in range
    return (coord.row() < num_grid_rows) && (coord.col() < num_grid_cols);
}

void ThetaStarPathPlanner::findAllBlockedCoords()
{
    for (auto &obstacle : obstacles)
    {
        auto blocked_points = obstacle->rasterize(SIZE_OF_GRID_CELL_IN_METERS);

        for (Point &blocked_point : blocked_points)
        {
            blocked_grid.insert(convertPointToCoord(blocked_point));
        }
    }
}

bool ThetaStarPathPlanner::isUnblocked(const Coordinate &coord)
{
    // If we haven't checked this Coordinate for obstacles before, check it now

    auto unblocked_grid_it = unblocked_grid.find(coord);
    if (unblocked_grid_it == unblocked_grid.end())
    {
        bool blocked = false;

        Point p = convertCoordToPoint(coord);
        for (auto &obstacle : obstacles)
        {
            if (obstacle->contains(p))
            {
                blocked = true;
                break;
            }
        }

        // We use the opposite convention to indicate blocked or not
        unblocked_grid[coord] = !blocked;
        return !blocked;
    }

    return unblocked_grid_it->second;
}

bool ThetaStarPathPlanner::isBlocked(const Coordinate &coord)
{
    auto blocked_grid_it = blocked_grid.find(coord);
    return blocked_grid_it != blocked_grid.end();
}

double ThetaStarPathPlanner::coordDistance(const Coordinate &coord1,
                                           const Coordinate &coord2) const
{
    Point p1(coord1.row(), coord1.col());
    Point p2(coord2.row(), coord2.col());
    return distance(p1, p2);
}

bool ThetaStarPathPlanner::lineOfSight(const Coordinate &coord1, const Coordinate &coord2)
{
    CoordinatePair coord_pair(coord1, coord2);
    // If we haven't checked this Coordinate pair for intersects before, check it now
    auto line_of_sight_cache_it = line_of_sight_cache.find(coord_pair);
    if (line_of_sight_cache_it == line_of_sight_cache.end())
    {
        Segment seg(convertCoordToPoint(coord1), convertCoordToPoint(coord2));
        bool has_line_of_sight = true;

        for (const auto &obstacle : obstacles)
        {
            if (obstacle->intersects(seg)) // TODO Could remove this after implementing findAllBlockedCoords
            {
                has_line_of_sight = false;
                break;
            }
        }

        // We use the opposite convention to indicate blocked or not
        line_of_sight_cache[coord_pair] = has_line_of_sight;
        return has_line_of_sight;
    }

    return line_of_sight_cache_it->second;
}

std::vector<Point> ThetaStarPathPlanner::tracePath(const Coordinate &end) const
{
    Coordinate current = end;
    std::vector<Point> path_points;
    std::stack<Coordinate> path;

    // loop until parent equals current
    while (!(cell_heuristics[current.row()][current.col()].parent() == current))
    {
        path.push(current);
        current = cell_heuristics[current.row()][current.col()].parent();
    }

    path.push(current);
    while (!path.empty())
    {
        Coordinate p = path.top();
        path.pop();
        path_points.push_back(convertCoordToPoint(p));
    }

    return path_points;
}

bool ThetaStarPathPlanner::updateVertex(const Coordinate &current, const Coordinate &next,
                                        const Coordinate &end)
{
    // Only process this CellHeuristic if this is a navigable one
    if (isCoordNavigable(next))
    {
        // If the successor is already on the closed list or if it is blocked, then ignore
        // it.  Else do the following
        if (closed_list.find(next) == closed_list.end() && !isBlocked(next))
        {
            double updated_best_path_cost;
            Coordinate next_parent;
            Coordinate parent = cell_heuristics[current.row()][current.col()].parent();
            if (lineOfSight(parent, next))
            {
                next_parent = parent;
                updated_best_path_cost =
                    cell_heuristics[parent.row()][parent.col()].bestPathCost() +
                    coordDistance(parent, next);
            }
            else
            {
                next_parent = current;
                updated_best_path_cost =
                    cell_heuristics[current.row()][current.col()].bestPathCost() +
                    coordDistance(current, next);
            }

            double next_start_to_end_cost_estimate =
                updated_best_path_cost + coordDistance(next, end);

            // If it isn’t on the open list, add it to the open list. Make the current
            // square the parent of this square. Record start_to_end_cost_estimate,
            // and best_path_cost of the square CellHeuristic
            //                               OR
            // If it is on the open list already, check to see if this path to that square
            // is better, using start_to_end_cost_estimate as the measure.
            if (!cell_heuristics[next.row()][next.col()].isInitialized() ||
                cell_heuristics[next.row()][next.col()].pathCostAndEndDistHeuristic() >
                    next_start_to_end_cost_estimate)
            {
                open_list.insert(std::make_pair(next_start_to_end_cost_estimate, next));

                // Update the details of this CellHeuristic
                cell_heuristics[next.row()][next.col()].update(
                    next_parent, next_start_to_end_cost_estimate, updated_best_path_cost);
            }
            // If the end is the same as the current successor
            if (next == end)
            {
                return true;
            }
        }
    }
    return false;
}


std::optional<Path> ThetaStarPathPlanner::findPath(
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

    resetAndInitializeMemberVariables(navigable_area, obstacles);

    Point closest_end      = findClosestFreePoint(end);
    Coordinate start_coord = convertPointToCoord(start);
    Coordinate end_coord   = convertPointToCoord(closest_end);

    bool no_path_exists = adjustEndPointsAndCheckForNoPath(start_coord, end_coord);
    if (no_path_exists)
    {
        return std::nullopt;
    }

    // if the start and end points are close enough, then return a straightline path
    if ((start - end).length() < CLOSE_TO_END_THRESHOLD ||
        ((std::abs(start.x() - end.x()) < SIZE_OF_GRID_CELL_IN_METERS) &&
         std::abs(start.y() - end.y()) < SIZE_OF_GRID_CELL_IN_METERS))
    {
        return Path(std::vector<Point>({start, end}));
    }
    if ((start - closest_end).length() <
            (CLOSE_TO_END_THRESHOLD * BLOCKED_END_OSCILLATION_MITIGATION) ||
        start_coord == end_coord)
    {
        return Path(std::vector<Point>({start, closest_end}));
    }

    // Initialising the parameters of the starting cell
    cell_heuristics[start_coord.row()][start_coord.col()].update(start_coord, 0.0, 0.0);
    open_list.insert(std::make_pair(0.0, start_coord));

    findAllBlockedCoords();

    bool found_end = findPathToEnd(end_coord);

    if (found_end == false)
    {
        return std::nullopt;
    }

    auto path_points = tracePath(end_coord);

    // The last point of path_points is the closest point on the grid to the end point, so
    // we need to replace that point with actual end point
    path_points.pop_back();
    path_points.push_back(closest_end);

    // The first point of path_points is the closest unblocked point on the grid to the
    // start point, so we need to replace that point with actual start point
    path_points.erase(path_points.begin());
    path_points.insert(path_points.begin(), start);

    return Path(path_points);
}

bool ThetaStarPathPlanner::adjustEndPointsAndCheckForNoPath(Coordinate &start_coord,
                                                            Coordinate &end_coord)
{
    bool ret_no_path = false;

    // If the source is out of range
    if (isCoordNavigable(start_coord) == false)
    {
        LOG(WARNING) << "Source is not within navigable area; no path found" << std::endl;
        ret_no_path = true;
    }

    // If the end is out of range
    if (isCoordNavigable(end_coord) == false)
    {
        LOG(WARNING) << "End is not within navigable area; no path found" << std::endl;
        ret_no_path = true;
    }

    // The source is blocked
    if (isUnblocked(start_coord) == false)
    {
        auto closest_start_coord = findClosestUnblockedCell(start_coord);
        if (closest_start_coord)
        {
            start_coord = *closest_start_coord;
        }
        else
        {
            ret_no_path = true;
        }
    }

    // The end is blocked
    if (isUnblocked(end_coord) == false)
    {
        auto closest_end_coord = findClosestUnblockedCell(end_coord);
        if (closest_end_coord)
        {
            end_coord = *closest_end_coord;
        }
        else
        {
            ret_no_path = true;
        }
    }

    return ret_no_path;
}

bool ThetaStarPathPlanner::findPathToEnd(const Coordinate &end_coord)
{
    while (!open_list.empty())
    {
        Coordinate current_coord(open_list.begin()->second);

        // Remove this vertex from the open list
        open_list.erase(open_list.begin());

        // Add this vertex to the closed list
        closed_list.insert(current_coord);

        // Check if the the destination is in the neighbouring coordinates
        if (visitNeighbours(current_coord, end_coord))
        {
            return true;
        }
    }

    // When the end CellHeuristic is not found and the open list is empty, then we
    // conclude that we failed to reach the end CellHeuristic. This may happen when the
    // there is no way to end CellHeuristic (due to blockages)
    return false;
}

bool ThetaStarPathPlanner::visitNeighbours(const Coordinate &current_coord,
                                           const Coordinate &end_coord)
{
    /*
        Generating all the 8 successor of this CellHeuristic

        Popped Cell --> (i, j)
        <0,+y>      --> (i-1, j)
        <0,-y>      --> (i+1, j)
        <+x,0>      --> (i, j+1)
        <-x,0>      --> (i, j-1)
        <+x,+y>     --> (i-1, j+1)
        <-x,+y>     --> (i-1, j-1)
        <+x,-y>     --> (i+1, j+1)
        <-x,-y>     --> (i+1, j-1)
        */

    Coordinate next_coord;
    unsigned int i = current_coord.row();
    unsigned int j = current_coord.col();

    for (int x_offset : {-1, 0, 1})
    {
        for (int y_offset : {-1, 0, 1})
        {
            next_coord = Coordinate(i + x_offset, j + y_offset);
            // check for clipping obstacles
            if (lineOfSight(current_coord, next_coord))
            {
                if (updateVertex(current_coord, next_coord, end_coord))
                {
                    return true;
                }
            }
        }
    }
    return false;
}

std::optional<ThetaStarPathPlanner::Coordinate>
ThetaStarPathPlanner::findClosestUnblockedCell(const Coordinate &current_cell)
{
    // spiral out from current_cell looking for unblocked cells
    unsigned int i = current_cell.row();
    unsigned int j = current_cell.col();
    Coordinate test_coord;
    unsigned next_index, curr_index = 3;
    int next_increment[4] = {1, 0, -1, 0};
    for (unsigned int depth = 1; depth < num_grid_rows; depth++)
    {
        next_index = (curr_index + 1) % 4;
        i += next_increment[next_index] * depth;
        j += next_increment[curr_index] * depth;
        test_coord = Coordinate(i, j);
        if (isCoordNavigable(test_coord) && isUnblocked(test_coord))
        {
            return test_coord;
        }
        curr_index = next_index;

        next_index = (curr_index + 1) % 4;
        i += next_increment[next_index] * depth;
        j += next_increment[curr_index] * depth;
        test_coord = Coordinate(i, j);
        if (isCoordNavigable(test_coord) && isUnblocked(test_coord))
        {
            return test_coord;
        }
        curr_index = next_index;
    }

    return std::nullopt;
}

Point ThetaStarPathPlanner::findClosestFreePoint(const Point &p)
{
    // expanding a circle to search for free points
    if (!isPointNavigableAndFreeOfObstacles(p))
    {
        int xc = static_cast<int>(p.x() * BLOCKED_END_SEARCH_RESOLUTION);
        int yc = static_cast<int>(p.y() * BLOCKED_END_SEARCH_RESOLUTION);

        for (int r = 1; r < max_navigable_x_coord * 2.0 * BLOCKED_END_SEARCH_RESOLUTION;
             r++)
        {
            int x = 0, y = r;
            int d = 3 - 2 * r;

            for (int outer : {-1, 1})
            {
                for (int inner : {-1, 1})
                {
                    Point p1 = Point(static_cast<double>(xc + outer * x) /
                                         BLOCKED_END_SEARCH_RESOLUTION,
                                     static_cast<double>(yc + inner * y) /
                                         BLOCKED_END_SEARCH_RESOLUTION);
                    Point p2 = Point(static_cast<double>(xc + outer * y) /
                                         BLOCKED_END_SEARCH_RESOLUTION,
                                     static_cast<double>(yc + inner * x) /
                                         BLOCKED_END_SEARCH_RESOLUTION);
                    if (isPointNavigableAndFreeOfObstacles(p1))
                    {
                        return p1;
                    }
                    if (isPointNavigableAndFreeOfObstacles(p2))
                    {
                        return p2;
                    }
                }
            }

            while (y >= x)
            {
                x++;

                // check for decision parameter
                // and correspondingly
                // update d, x, y
                if (d > 0)
                {
                    y--;
                    d = d + 4 * (x - y) + 10;
                }
                else
                {
                    d = d + 4 * x + 6;
                }

                for (int outer : {-1, 1})
                {
                    for (int inner : {-1, 1})
                    {
                        Point p1 =
                            Point((xc + outer * x) / BLOCKED_END_SEARCH_RESOLUTION,
                                  (yc + inner * y) / BLOCKED_END_SEARCH_RESOLUTION);
                        Point p2 =
                            Point((xc + outer * y) / BLOCKED_END_SEARCH_RESOLUTION,
                                  (yc + inner * x) / BLOCKED_END_SEARCH_RESOLUTION);
                        if (isPointNavigableAndFreeOfObstacles(p1))
                        {
                            return p1;
                        }
                        if (isPointNavigableAndFreeOfObstacles(p2))
                        {
                            return p2;
                        }
                    }
                }
            }
        }
    }
    return p;
}

bool ThetaStarPathPlanner::isPointNavigableAndFreeOfObstacles(const Point &p)
{
    if (!isPointNavigable(p))
    {
        return false;
    }

    // TODO is this checking isUnblocked?
    for (auto &obstacle : obstacles)
    {
        if (obstacle->contains(p))
        {
            return false;
        }
    }

    return true;
}

bool ThetaStarPathPlanner::isPointNavigable(const Point &p) const
{
    return ((p.x() > -max_navigable_x_coord + centre.x()) &&
            (p.x() < max_navigable_x_coord + centre.x()) &&
            (p.y() > -max_navigable_y_coord + centre.y()) &&
            (p.y() < max_navigable_y_coord + centre.y()));
}


Point ThetaStarPathPlanner::convertCoordToPoint(const Coordinate &coord) const
{
    // account for robot radius
    return Point(
        (coord.row() * SIZE_OF_GRID_CELL_IN_METERS) - max_navigable_x_coord + centre.x(),
        (coord.col() * SIZE_OF_GRID_CELL_IN_METERS) - max_navigable_y_coord + centre.y());
}

ThetaStarPathPlanner::Coordinate ThetaStarPathPlanner::convertPointToCoord(
    const Point &p) const
{
    // account for robot radius
    return Coordinate(static_cast<int>((p.x() + max_navigable_x_coord - centre.x()) /
                                       SIZE_OF_GRID_CELL_IN_METERS),
                      static_cast<int>((p.y() + max_navigable_y_coord - centre.y()) /
                                       SIZE_OF_GRID_CELL_IN_METERS));
}

void ThetaStarPathPlanner::resetAndInitializeMemberVariables(
    const Rectangle &navigable_area, const std::vector<ObstaclePtr> &obstacles)
{
    // Initialize member variables
    this->obstacles = obstacles;
    centre          = navigable_area.centre();
    max_navigable_x_coord =
        std::max(navigable_area.xLength() / 2.0 - ROBOT_MAX_RADIUS_METERS, 0.0);
    max_navigable_y_coord =
        std::max(navigable_area.yLength() / 2.0 - ROBOT_MAX_RADIUS_METERS, 0.0);
    num_grid_rows =
        static_cast<int>((max_navigable_x_coord * 2.0 + ROBOT_MAX_RADIUS_METERS) /
                         SIZE_OF_GRID_CELL_IN_METERS);
    num_grid_cols =
        static_cast<int>((max_navigable_y_coord * 2.0 + ROBOT_MAX_RADIUS_METERS) /
                         SIZE_OF_GRID_CELL_IN_METERS);

    // add assertion to ensure that the key value in Coordinate and Coordinate pair would
    // not overflow, overflow would happen when grid row or col is larger than 1<<16
    assert(num_grid_rows < (1 << 16));
    assert(num_grid_cols < (1 << 16));

    // Reset data structures to path plan again
    open_list.clear();
    closed_list.clear();
    blocked_grid.clear();
    unblocked_grid.clear();
    line_of_sight_cache.clear();
    cell_heuristics = std::vector<std::vector<CellHeuristic>>(
        num_grid_rows,
        std::vector<CellHeuristic>(num_grid_cols, ThetaStarPathPlanner::CellHeuristic()));
}
