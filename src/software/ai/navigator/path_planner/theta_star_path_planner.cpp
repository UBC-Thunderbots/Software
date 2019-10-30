/**
 * This file contains the implementation of a theta star path planner
 * which returns an optimal path that avoids obstacles
 */

#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

#include <g3log/g3log.hpp>

bool ThetaStarPathPlanner::isValid(Coordinate test_coord)
{
    // Returns true if row number and column number
    // is in range
    return (test_coord.row() >= 0) && (test_coord.row() < num_grid_rows) &&
           (test_coord.col() >= 0) && (test_coord.col() < num_grid_cols);
}

bool ThetaStarPathPlanner::isUnBlocked(Coordinate test_coord)
{
    // If we haven't checked this Coordinate before, check it now
    if (unblocked_grid.find(test_coord) == unblocked_grid.end())
    {
        bool blocked = false;

        Point p = convertCoordinateToPoint(test_coord);
        for (auto &obstacle : obstacles)
        {
            if (obstacle.containsPoint(p))
            {
                blocked = true;
                break;
            }
        }

        // We use the opposite convention to indicate blocked or not
        unblocked_grid[test_coord] = !blocked;
    }

    return unblocked_grid[test_coord];
}

bool ThetaStarPathPlanner::isDestination(Coordinate test_coord, Coordinate dest)
{
    return (test_coord.row() == dest.row() && test_coord.col() == dest.col());
}

double ThetaStarPathPlanner::calculateHValue(Coordinate test_coord, Coordinate dest)
{
    // Return using the distance formula
    return (
        (double)sqrt((test_coord.row() - dest.row()) * (test_coord.row() - dest.row()) +
                     (test_coord.col() - dest.col()) * (test_coord.col() - dest.col())));
}

bool ThetaStarPathPlanner::lineOfSight(Coordinate current_parent, Coordinate new_coord)
{
    Point next_point     = Point(new_coord.row(), new_coord.col());
    Point parent_point   = Point(current_parent.row(), current_parent.col());
    Point point_to_check = parent_point;

    Vector diff      = next_point - parent_point;
    Vector direction = diff.norm();
    int dist         = (int)diff.len();
    for (int i = 0; i < dist; i++)
    {
        point_to_check = point_to_check + direction;
        if (!isUnBlocked(Coordinate((int)std::round(point_to_check.x()),
                                    (int)std::round(point_to_check.y()))))
        {
            return false;
        }
    }
    return true;
}

std::vector<Point> ThetaStarPathPlanner::tracePath(Coordinate dest)
{
    int row                        = dest.row();
    int col                        = dest.col();
    std::vector<Point> path_points = std::vector<Point>();

    std::stack<Coordinate> coord_path;

    // loop until parent is self
    while (!(cell_details[row][col].parent.row() == row &&
             cell_details[row][col].parent.col() == col))
    {
        coord_path.push(Coordinate(row, col));
        int temp_row = cell_details[row][col].parent.row();
        int temp_col = cell_details[row][col].parent.col();
        row          = temp_row;
        col          = temp_col;
    }

    coord_path.push(Coordinate(row, col));
    while (!coord_path.empty())
    {
        Coordinate p = coord_path.top();
        coord_path.pop();
        path_points.push_back(convertCoordinateToPoint(p));
    }

    return path_points;
}

bool ThetaStarPathPlanner::updateVertex(Coordinate current_coord, Coordinate new_coord,
                                        Coordinate dest, double curr_to_new_dist)
{
    // Only process this GridCell if this is a valid one
    if (isValid(new_coord) == true)
    {
        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        if (closed_list[new_coord.row()][new_coord.col()] == false &&
            isUnBlocked(new_coord) == true)
        {
            double g_new;
            Coordinate parent_new;
            Coordinate parent =
                cell_details[current_coord.row()][current_coord.col()].parent;
            if (lineOfSight(parent, new_coord))
            {
                parent_new = parent;
                g_new      = cell_details[parent.row()][parent.col()].g +
                        calculateHValue(parent, new_coord);
            }
            else
            {
                parent_new = current_coord;
                g_new      = cell_details[current_coord.row()][current_coord.col()].g +
                        curr_to_new_dist;
            }

            double h_new = calculateHValue(new_coord, dest);
            double f_new = g_new + h_new;

            // If it isn’t on the open list, add it to
            // the open list. Make the current square
            // the parent of this square. Record the
            // f, g, and h costs of the square GridCell
            //			 OR
            // If it is on the open list already, check
            // to see if this path to that square is better,
            // using 'f' cost as the measure.
            if (cell_details[new_coord.row()][new_coord.col()].f == DBL_MAX ||
                cell_details[new_coord.row()][new_coord.col()].f > f_new)
            {
                open_list.insert(
                    OpenListCell(f_new, Coordinate(new_coord.row(), new_coord.col())));

                // Update the details of this GridCell
                cell_details[new_coord.row()][new_coord.col()].f      = f_new;
                cell_details[new_coord.row()][new_coord.col()].g      = g_new;
                cell_details[new_coord.row()][new_coord.col()].h      = h_new;
                cell_details[new_coord.row()][new_coord.col()].parent = parent_new;
            }
            // If the destination GridCell is the same as the
            // current successor
            if (isDestination(new_coord, dest) == true)
            {
                return true;
            }
        }
    }
    return false;
}

// top level function
Path ThetaStarPathPlanner::findPath(const Point &start, const Point &destination,
                                    const Rectangle &navigable_area,
                                    const std::vector<Obstacle> &obstacles)
{
    // Initialize member variables
    this->obstacles       = obstacles;
    max_navigable_x_coord = navigable_area.xLength() / 2.0 - ROBOT_MAX_RADIUS_METERS;
    max_navigable_y_coord = navigable_area.yLength() / 2.0 - ROBOT_MAX_RADIUS_METERS;
    num_grid_rows = (int)((max_navigable_x_coord * 2.0 + ROBOT_MAX_RADIUS_METERS) /
                          SIZE_OF_GRID_CELL_IN_METERS);
    num_grid_cols = (int)((max_navigable_y_coord * 2.0 + ROBOT_MAX_RADIUS_METERS) /
                          SIZE_OF_GRID_CELL_IN_METERS);

    // Reset data structures to path plan again
    open_list.clear();
    unblocked_grid.clear();

    // Initialize local variables
    Point closest_destination = findClosestFreePoint(destination);
    Coordinate src_coord      = convertPointToCoordinate(start);
    Coordinate dest_coord     = convertPointToCoordinate(closest_destination);
    Path empty_ret_val(std::nullopt);

    // If the source is out of range
    if (isValid(src_coord) == false)
    {
        LOG(WARNING) << "Source is not valid; no path found" << std::endl;
        return empty_ret_val;
    }

    // If the destination is out of range
    if (isValid(dest_coord) == false)
    {
        LOG(WARNING) << "Destination is not valid; no path found" << std::endl;
        return empty_ret_val;
    }

    if ((start - destination).len() < CLOSE_TO_DEST_THRESHOLD ||
        ((start - destination).len() < SIZE_OF_GRID_CELL_IN_METERS))
    {
        // If the destination GridCell is within one grid size of start or
        // start and destination, or start and closest_destination, within threshold
        return Path(std::vector<Point>({start, destination}));
    }


    if ((start - closest_destination).len() <
        (CLOSE_TO_DEST_THRESHOLD * BLOCKED_DESINATION_OSCILLATION_MITIGATION))
    {
        return Path(std::vector<Point>({start, closest_destination}));
    }

    // The source is blocked
    if (isUnBlocked(src_coord) == false)
    {
        auto tmp_src_coord = findClosestUnblockedCell(src_coord);
        if (tmp_src_coord)
        {
            src_coord = *tmp_src_coord;
        }
        else
        {
            return empty_ret_val;
        }
    }

    // The destination is blocked
    if (isUnBlocked(dest_coord) == false)
    {
        auto tmp_dest_coord = findClosestUnblockedCell(dest_coord);
        if (tmp_dest_coord)
        {
            dest_coord = *tmp_dest_coord;
        }
        else
        {
            return empty_ret_val;
        }
    }

    closed_list = std::vector<std::vector<bool>>(num_grid_rows,
                                                 std::vector<bool>(num_grid_cols, false));

    cell_details = std::vector<std::vector<GridCell>>(
        num_grid_rows,
        std::vector<GridCell>(num_grid_cols,
                              ThetaStarPathPlanner::GridCell(Coordinate(-1, -1), DBL_MAX,
                                                             DBL_MAX, DBL_MAX)));

    int i, j;

    // Initialising the parameters of the starting node
    i = src_coord.row(), j = src_coord.col();
    cell_details[i][j].f      = 0.0;
    cell_details[i][j].g      = 0.0;
    cell_details[i][j].h      = 0.0;
    cell_details[i][j].parent = src_coord;

    // Put the starting GridCell on the open list and set its
    // 'f' as 0
    open_list.insert(OpenListCell(0.0, src_coord));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool found_dest = false;

    while (!open_list.empty())
    {
        OpenListCell p = *open_list.begin();

        // Remove this vertex from the open list
        open_list.erase(open_list.begin());

        // Add this vertex to the closed list
        i                 = p.second.row();
        j                 = p.second.col();
        closed_list[i][j] = true;

        /*
            Generating all the 8 successor of this GridCell

            Popped Cell --> (i, j)
            <0,+y>      --> (i-1, j)
            <0,-y>      --> (i+1, j)
            <+x,0>      --> (i, j+1)
            <-x,0>      --> (i, j-1)
            <+x,+y>     --> (i-1, j+1)
            <-x,+y>     --> (i-1, j-1)
            <+x,-y>     --> (i+1, j+1)
            <-x,-y>     --> (i+1, j-1)*/

        Coordinate current_coord, new_coord;
        current_coord = Coordinate(i, j);

        for (int x_offset : {-1, 0, 1})
        {
            for (int y_offset : {-1, 0, 1})
            {
                double dist_to_neighbour =
                    std::sqrt(std::pow(x_offset, 2) + std::pow(y_offset, 2));
                new_coord = Coordinate(i + x_offset, j + y_offset);
                found_dest =
                    updateVertex(current_coord, new_coord, dest_coord, dist_to_neighbour);
                if (found_dest)
                {
                    goto loop_end;
                }
            }
        }
    }

loop_end:

    // When the destination GridCell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destination GridCell. This may happen when the
    // there is no way to destination GridCell (due to blockages)
    if (found_dest == false)
    {
        return empty_ret_val;
    }

    auto path_points = tracePath(dest_coord);

    // replace destination with actual destination
    path_points.pop_back();
    path_points.push_back(closest_destination);

    // replace src with actual start
    path_points.erase(path_points.begin());
    path_points.insert(path_points.begin(), start);

    return Path(path_points);
}

std::optional<ThetaStarPathPlanner::Coordinate>
ThetaStarPathPlanner::findClosestUnblockedCell(Coordinate current_cell)
{
    // spiral out from current_cell looking for unblocked cells
    int i = current_cell.row();
    int j = current_cell.col();
    Coordinate test_coord;
    unsigned next_index, curr_index = 3;
    int next_increment[4] = {1, 0, -1, 0};
    for (int depth = 1; depth < num_grid_rows; depth++)
    {
        next_index = (curr_index + 1) % 4;
        i += next_increment[next_index] * depth;
        j += next_increment[curr_index] * depth;
        test_coord = Coordinate(i, j);
        if (isValid(test_coord) && isUnBlocked(test_coord))
        {
            return test_coord;
        }
        curr_index = next_index;

        next_index = (curr_index + 1) % 4;
        i += next_increment[next_index] * depth;
        j += next_increment[curr_index] * depth;
        test_coord = Coordinate(i, j);
        if (isValid(test_coord) && isUnBlocked(test_coord))
        {
            return test_coord;
        }
        curr_index = next_index;
    }

    return std::nullopt;
}

Point ThetaStarPathPlanner::findClosestFreePoint(Point p)
{
    if (!isValidAndFreeOfObstacles(p))
    {
        int xc = (int)(p.x() * BLOCKED_DESTINATION_SEARCH_RESOLUTION);
        int yc = (int)(p.y() * BLOCKED_DESTINATION_SEARCH_RESOLUTION);

        for (int r = 1;
             r < max_navigable_x_coord * 2.0 * BLOCKED_DESTINATION_SEARCH_RESOLUTION; r++)
        {
            int x = 0, y = r;
            int d = 3 - 2 * r;

            for (int outer : {-1, 1})
            {
                for (int inner : {-1, 1})
                {
                    Point p1 = Point(
                        (double)(xc + outer * x) / BLOCKED_DESTINATION_SEARCH_RESOLUTION,
                        (double)(yc + inner * y) / BLOCKED_DESTINATION_SEARCH_RESOLUTION);
                    Point p2 = Point(
                        (double)(xc + outer * y) / BLOCKED_DESTINATION_SEARCH_RESOLUTION,
                        (double)(yc + inner * x) / BLOCKED_DESTINATION_SEARCH_RESOLUTION);
                    if (isValidAndFreeOfObstacles(p1))
                    {
                        return p1;
                    }
                    if (isValidAndFreeOfObstacles(p2))
                    {
                        return p2;
                    }
                }
            }

            while (y >= x)
            {
                // for each pixel we will
                // draw all eight pixels

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
                        Point p1 = Point(
                            (xc + outer * x) / BLOCKED_DESTINATION_SEARCH_RESOLUTION,
                            (yc + inner * y) / BLOCKED_DESTINATION_SEARCH_RESOLUTION);
                        Point p2 = Point(
                            (xc + outer * y) / BLOCKED_DESTINATION_SEARCH_RESOLUTION,
                            (yc + inner * x) / BLOCKED_DESTINATION_SEARCH_RESOLUTION);
                        if (isValidAndFreeOfObstacles(p1))
                        {
                            return p1;
                        }
                        if (isValidAndFreeOfObstacles(p2))
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

bool ThetaStarPathPlanner::isValidAndFreeOfObstacles(Point p)
{
    if ((p.x() > -max_navigable_x_coord) && (p.x() < max_navigable_x_coord) &&
        (p.y() > -max_navigable_y_coord) && (p.y() < max_navigable_y_coord))
    {
        for (auto &obstacle : obstacles)
        {
            if (obstacle.containsPoint(p))
            {
                return false;
            }
        }
        return true;
    }

    return false;
}

Point ThetaStarPathPlanner::convertCoordinateToPoint(Coordinate coord)
{
    // account for robot radius
    return Point((coord.row() * SIZE_OF_GRID_CELL_IN_METERS) - max_navigable_x_coord,
                 (coord.col() * SIZE_OF_GRID_CELL_IN_METERS) - max_navigable_y_coord);
}

ThetaStarPathPlanner::Coordinate ThetaStarPathPlanner::convertPointToCoordinate(Point p)
{
    // account for robot radius
    return Coordinate(
        (int)((p.x() + max_navigable_x_coord) / SIZE_OF_GRID_CELL_IN_METERS),
        (int)((p.y() + max_navigable_y_coord) / SIZE_OF_GRID_CELL_IN_METERS));
}
