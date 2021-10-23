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
            Coordinate blocked_coord = convertPointToCoord(blocked_point);
            if (isCoordNavigable(blocked_coord))
            {
                blocked_grid[blocked_coord.row()][blocked_coord.col()] = true;
            }
        }
    }
}

bool ThetaStarPathPlanner::isBlocked(const Coordinate &coord)
{
    return blocked_grid[coord.row()][coord.col()];
}

double ThetaStarPathPlanner::coordDistance(const Coordinate &coord1,
                                           const Coordinate &coord2) const
{
    Point p1(coord1.row(), coord1.col());
    Point p2(coord2.row(), coord2.col());
    return distance(p1, p2);
}

bool ThetaStarPathPlanner::lineOfSight(const Coordinate &coord0, const Coordinate &coord1)
{
    int dy = coord1.col() - coord0.col();
    int dx = coord1.row() - coord0.row();

    if (std::abs(dy) < std::abs(dx))
    {
        if (coord0.row() > coord1.row())
        {
            return checkLine(coord1, coord0, true);
        }
        else
        {
            return checkLine(coord0, coord1, true);
        }
    }
    else
    {
        if (coord0.col() > coord1.col())
        {
            return checkLine(coord1, coord0, false);
        }
        else
        {
            return checkLine(coord0, coord1, false);
        }
    }
}

bool ThetaStarPathPlanner::checkLine(const Coordinate &coord0, const Coordinate &coord1,
                                     const bool isLineLow)
{
    // Main represents the axis that is being incremented (x if line is low)
    // Sec represents the secondary axis that is dependent on Main axis (y if line is low)
    int main0;
    int sec0;
    int main1;
    int sec1;
    if (isLineLow)
    {
        main0 = static_cast<int>(coord0.row());
        sec0  = static_cast<int>(coord0.col());
        main1 = static_cast<int>(coord1.row());
        sec1  = static_cast<int>(coord1.col());
    }
    else
    {
        sec0  = static_cast<int>(coord0.row());
        main0 = static_cast<int>(coord0.col());
        sec1  = static_cast<int>(coord1.row());
        main1 = static_cast<int>(coord1.col());
    }

    int d_main = main1 - main0;
    int d_sec  = sec1 - sec0;
    int sec_i  = 1;

    if (d_sec < 0)
    {
        sec_i = -1;
        d_sec = -d_sec;
    }
    int D   = (2 * d_sec) - d_main;
    int sec = sec0;

    for (int main = main0; main <= main1; main++)
    {
        Coordinate curr_coord = isLineLow ? Coordinate(main, sec) : Coordinate(sec, main);
        if (isBlocked(curr_coord))
        {
            // No line of sight since a coordinate in the path from coord0 to coord1 is
            // blocked
            return false;
        }
        if (D > 0)
        {
            sec += sec_i;
            D += 2 * (d_sec - d_main);
        }
        else
        {
            D += 2 * d_sec;
        }
    }
    return true;
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

    findAllBlockedCoords();

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
    if ((start - closest_end).length() < CLOSE_TO_END_THRESHOLD ||
        start_coord == end_coord)
    {
        return Path(std::vector<Point>({start, closest_end}));
    }

    // Initialising the parameters of the starting cell
    cell_heuristics[start_coord.row()][start_coord.col()].update(start_coord, 0.0, 0.0);
    open_list.insert(std::make_pair(0.0, start_coord));

    // Avoiding the situation where closest_end point is free but end_coord is blocked
    blocked_grid[start_coord.row()][start_coord.col()] = false;
    blocked_grid[end_coord.row()][end_coord.col()]     = false;

    bool found_end = findPathToEnd(end_coord);

    if (found_end == false)
    {
        return std::nullopt;
    }

    auto path_points = tracePath(end_coord);

    // The last point of path_points is the closest point on the grid to the end point, so
    // we need to replace that point with actual end point
    path_points.pop_back();
    if (path_points.back() != closest_end)
    {
        path_points.push_back(closest_end);
    }

    // The first point of path_points is the closest unblocked point on the grid to the
    // start point, so we need to replace that point with actual start point
    path_points.erase(path_points.begin());
    path_points.insert(path_points.begin(), start);

    if (path_points.size() > 2 &&
        (path_points[0] - path_points[1]).length() < SIZE_OF_GRID_CELL_IN_METERS)
    {
        path_points.erase(path_points.begin() + 1);
    }

    return Path(path_points);
}

bool ThetaStarPathPlanner::adjustEndPointsAndCheckForNoPath(Coordinate &start_coord,
                                                            Coordinate &end_coord)
{
    bool ret_no_path = false;

    // If the source is out of range
    if (!isCoordNavigable(start_coord))
    {
        LOG(WARNING) << "Source is not within navigable area; no path found" << std::endl;
        ret_no_path = true;
    }

    // If the end is out of range
    if (!isCoordNavigable(end_coord))
    {
        LOG(WARNING) << "End is not within navigable area; no path found" << std::endl;
        ret_no_path = true;
    }

    // The source is blocked
    if (isBlocked(start_coord))
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
    if (isBlocked(end_coord))
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

            // Only check points in the navigable area
            if (!isCoordNavigable(next_coord))
            {
                continue;
            }

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
    // Check the horizontal and vertical cells relative to current_cell for the nearest
    // unblocked cell
    unsigned int i = current_cell.row();
    unsigned int j = current_cell.col();
    Coordinate test_coord;
    int depth_sign = 1;
    for (unsigned int depth = 1; depth < num_grid_rows; depth++)
    {
        test_coord = Coordinate(i + depth * depth_sign, j);
        if (isCoordNavigable(test_coord) && !isBlocked(test_coord))
        {
            return test_coord;
        }

        test_coord = Coordinate(i, j + depth * depth_sign);
        if (isCoordNavigable(test_coord) && !isBlocked(test_coord))
        {
            return test_coord;
        }

        depth_sign *= -1;
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

    Coordinate test_coord = convertPointToCoord(p);
    if (isBlocked(test_coord))
    {
        return false;
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
    line_of_sight_cache.clear();
    cell_heuristics = std::vector<std::vector<CellHeuristic>>(
        num_grid_rows,
        std::vector<CellHeuristic>(num_grid_cols, ThetaStarPathPlanner::CellHeuristic()));
    blocked_grid = std::vector<std::vector<bool>>(
        num_grid_rows, std::vector<bool>(num_grid_cols, false));
}
