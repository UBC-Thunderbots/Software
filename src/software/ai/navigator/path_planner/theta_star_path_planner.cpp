/**
 * This file contains the implementation of a theta star path planner
 * which returns an optimal path that avoids obstacles
 */

#include "software/ai/navigator/path_planner/theta_star_path_planner.h"

#include <g3log/g3log.hpp>

bool ThetaStarPathPlanner::isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < numGridRows) && (col >= 0) && (col < numGridCols);
}

bool ThetaStarPathPlanner::isUnBlocked(int row, int col)
{
    auto cell = std::pair<int, int>(row, col);

    // If we haven't checked this cell before, check it now
    if (unblocked_grid.find(cell) == unblocked_grid.end())
    {
        bool blocked = false;

        Point p = convertCellToPoint(row, col);
        for (auto &obstacle : obstacles)
        {
            if (obstacle.containsPoint(p))
            {
                blocked = true;
                break;
            }
        }

        // We use the opposite convention to indicate blocked or not
        unblocked_grid[cell] = !blocked;
    }

    return unblocked_grid[cell];
}

bool ThetaStarPathPlanner::isDestination(int row, int col, GridPoint dest)
{
    return (row == dest.first && col == dest.second);
}

double ThetaStarPathPlanner::calculateHValue(int row, int col, GridPoint dest)
{
    // Return using the distance formula
    return ((double)sqrt((row - dest.first) * (row - dest.first) +
                         (col - dest.second) * (col - dest.second)));
}

bool ThetaStarPathPlanner::lineOfSight(int curr_parent_i, int curr_parent_j,
                                       GridPoint new_pair)
{
    Point nextPoint    = Point(new_pair.first, new_pair.second);
    Point parentPoint  = Point(curr_parent_i, curr_parent_j);
    Point pointToCheck = parentPoint;

    Vector diff      = nextPoint - parentPoint;
    Vector direction = diff.norm();
    int dist         = (int)diff.len();
    for (int i = 0; i < dist; i++)
    {
        pointToCheck = pointToCheck + direction;
        if (!isUnBlocked((int)std::round(pointToCheck.x()),
                         (int)std::round(pointToCheck.y())))
        {
            return false;
        }
    }
    return true;
}

std::vector<Point> ThetaStarPathPlanner::tracePath(GridPoint dest)
{
    int row                        = dest.first;
    int col                        = dest.second;
    std::vector<Point> path_points = std::vector<Point>();

    std::stack<GridPoint> Path;

    // loop until parent is self
    while (!(cellDetails[row][col].parent.first == row &&
             cellDetails[row][col].parent.second == col))
    {
        Path.push(GridPoint(row, col));
        int temp_row = cellDetails[row][col].parent.first;
        int temp_col = cellDetails[row][col].parent.second;
        row          = temp_row;
        col          = temp_col;
    }

    Path.push(GridPoint(row, col));
    while (!Path.empty())
    {
        GridPoint p = Path.top();
        Path.pop();
        path_points.push_back(convertCellToPoint(p.first, p.second));
    }

    return path_points;
}

bool ThetaStarPathPlanner::updateVertex(GridPoint pCurr, GridPoint pNew, GridPoint dest,
                                        double currToNextNodeDist)
{
    // Only process this GridCell if this is a valid one
    if (isValid(pNew.first, pNew.second) == true)
    {
        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        if (closedList[pNew.first][pNew.second] == false &&
            isUnBlocked(pNew.first, pNew.second) == true)
        {
            double gNew;
            GridPoint parent_new;
            GridPoint parent = cellDetails[pCurr.first][pCurr.second].parent;
            if (lineOfSight(parent.first, parent.second, pNew))
            {
                parent_new = parent;
                gNew       = cellDetails[parent.first][parent.second].g_ +
                       calculateHValue(parent.first, parent.second, pNew);
            }
            else
            {
                parent_new = pCurr;
                gNew = cellDetails[pCurr.first][pCurr.second].g_ + currToNextNodeDist;
            }

            double hNew = calculateHValue(pNew.first, pNew.second, dest);
            double fNew = gNew + hNew;

            // If it isn’t on the open list, add it to
            // the open list. Make the current square
            // the parent of this square. Record the
            // f, g, and h costs of the square GridCell
            //			 OR
            // If it is on the open list already, check
            // to see if this path to that square is better,
            // using 'f' cost as the measure.
            if (cellDetails[pNew.first][pNew.second].f_ == DBL_MAX ||
                cellDetails[pNew.first][pNew.second].f_ > fNew)
            {
                openList.insert(OpenListCell(fNew, GridPoint(pNew.first, pNew.second)));

                // Update the details of this GridCell
                cellDetails[pNew.first][pNew.second].f_        = fNew;
                cellDetails[pNew.first][pNew.second].g_        = gNew;
                cellDetails[pNew.first][pNew.second].h_        = hNew;
                cellDetails[pNew.first][pNew.second].parent    = parent_new;
            }
            // If the destination GridCell is the same as the
            // current successor
            if (isDestination(pNew.first, pNew.second, dest) == true)
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
    this->obstacles = obstacles;
    maxNavXCoord    = navigable_area.xLength() / 2.0 - ROBOT_MAX_RADIUS_METERS;
    maxNavYCoord    = navigable_area.yLength() / 2.0 - ROBOT_MAX_RADIUS_METERS;
    numGridRows     = (int)((maxNavXCoord * 2.0 + ROBOT_MAX_RADIUS_METERS) /
                        SIZE_OF_GRID_CELL_IN_METERS);
    numGridCols     = (int)((maxNavYCoord * 2.0 + ROBOT_MAX_RADIUS_METERS) /
                        SIZE_OF_GRID_CELL_IN_METERS);

    // Reset data structures to path plan again
    openList.clear();
    unblocked_grid.clear();

    // Initialize local variables
    Point closest_destination = findClosestFreePoint(destination);
    GridPoint src_coord       = convertPointToCell(start);
    GridPoint dest_coord      = convertPointToCell(closest_destination);
    Path empty_ret_val(std::vector<Point>({}));

    // If the source is out of range
    if (isValid(src_coord.first, src_coord.second) == false)
    {
        LOG(WARNING) << "Source is not valid; no path found" << std::endl;
        return empty_ret_val;
    }

    // If the destination is out of range
    if (isValid(dest_coord.first, dest_coord.second) == false)
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
    if (isUnBlocked(src_coord.first, src_coord.second) == false)
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
    if (isUnBlocked(dest_coord.first, dest_coord.second) == false)
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

    closedList = std::vector<std::vector<bool>>(numGridRows,
                                                std::vector<bool>(numGridCols, false));

    cellDetails = std::vector<std::vector<GridCell>>(
        numGridRows, std::vector<GridCell>(
                         numGridCols, ThetaStarPathPlanner::GridCell(
                                          GridPoint(-1, -1), DBL_MAX, DBL_MAX, DBL_MAX)));

    int i, j;

    // Initialising the parameters of the starting node
    i = src_coord.first, j = src_coord.second;
    cellDetails[i][j].f_        = 0.0;
    cellDetails[i][j].g_        = 0.0;
    cellDetails[i][j].h_        = 0.0;
    cellDetails[i][j].parent    = src_coord;

    // Put the starting GridCell on the open list and set its
    // 'f' as 0
    openList.insert(OpenListCell(0.0, src_coord));

    // We set this boolean value as false as initially
    // the destination is not reached.
    bool foundDest = false;

    while (!openList.empty())
    {
        OpenListCell p = *openList.begin();

        // Remove this vertex from the open list
        openList.erase(openList.begin());

        // Add this vertex to the closed list
        i                = p.second.first;
        j                = p.second.second;
        closedList[i][j] = true;

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

        GridPoint pCurr, pNew;
        pCurr = GridPoint(i, j);

        for (int x_offset : {-1, 0, 1})
        {
            for (int y_offset : {-1, 0, 1})
            {
                double dist_to_neighbour =
                    std::sqrt(std::pow(x_offset, 2) + std::pow(y_offset, 2));
                pNew      = GridPoint(i + x_offset, j + y_offset);
                foundDest = updateVertex(pCurr, pNew, dest_coord, dist_to_neighbour);
                if (foundDest)
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
    if (foundDest == false)
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

std::optional<ThetaStarPathPlanner::GridPoint>
ThetaStarPathPlanner::findClosestUnblockedCell(GridPoint currCell)
{
    // spiral out from currCell looking for unblocked cells
    int i = currCell.first;
    int j = currCell.second;
    unsigned nextIndex, currIndex = 3;
    int nextIncrement[4] = {1, 0, -1, 0};
    for (int depth = 1; depth < numGridRows; depth++)
    {
        nextIndex = (currIndex + 1) % 4;
        i += nextIncrement[nextIndex] * depth;
        j += nextIncrement[currIndex] * depth;
        if (isValid(i, j) && isUnBlocked(i, j))
        {
            return GridPoint(i, j);
        }
        currIndex = nextIndex;

        nextIndex = (currIndex + 1) % 4;
        i += nextIncrement[nextIndex] * depth;
        j += nextIncrement[currIndex] * depth;
        if (isValid(i, j) && isUnBlocked(i, j))
        {
            return GridPoint(i, j);
        }
        currIndex = nextIndex;
    }

    return std::nullopt;
}

Point ThetaStarPathPlanner::findClosestFreePoint(Point p)
{
    if (!isValidAndFreeOfObstacles(p))
    {
        int xc = (int)(p.x() * BLOCKED_DESTINATION_SEARCH_RESOLUTION);
        int yc = (int)(p.y() * BLOCKED_DESTINATION_SEARCH_RESOLUTION);

        for (int r = 1; r < maxNavXCoord * 2.0 * BLOCKED_DESTINATION_SEARCH_RESOLUTION;
             r++)
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
    if ((p.x() > -maxNavXCoord) && (p.x() < maxNavXCoord) && (p.y() > -maxNavYCoord) &&
        (p.y() < maxNavYCoord))
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

Point ThetaStarPathPlanner::convertCellToPoint(int row, int col)
{
    // account for robot radius
    return Point((row * SIZE_OF_GRID_CELL_IN_METERS) - maxNavXCoord,
                 (col * SIZE_OF_GRID_CELL_IN_METERS) - maxNavYCoord);
}

ThetaStarPathPlanner::GridPoint ThetaStarPathPlanner::convertPointToCell(Point p)
{
    // account for robot radius
    return GridPoint((int)((p.x() + maxNavXCoord) / SIZE_OF_GRID_CELL_IN_METERS),
                     (int)((p.y() + maxNavYCoord) / SIZE_OF_GRID_CELL_IN_METERS));
}
