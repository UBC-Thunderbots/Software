#include "ai/navigator/path_planner/theta_star_path_planner.h"

/**
 * This file contains the implementation of a theta star path planner
 * which returns an optimal path that avoids obstacles
 */


ThetaStarPathPlanner::ThetaStarPathPlanner(Field field, Ball ball,
                                           const std::vector<Obstacle> &obstacles)
    : field_(field), ball_(ball), obstacles_(obstacles)
{
    numRows = (int)field_.length() / GRID_DIVISION_IN_METERS;
    numCols = (int)field_.width() / GRID_DIVISION_IN_METERS;

    unblocked_grid =
        std::vector<std::vector<bool>>(numRows, std::vector<bool>(numCols, true));

    for (unsigned row = 0; row < numRows; row++)
    {
        for (unsigned col = 0; col < numCols; col++)
        {
            Point p(col * GRID_DIVISION_IN_METERS - field_.length() / 2,
                    row * GRID_DIVISION_IN_METERS - field_.width() / 2);
            for (unsigned index = 0; index < obstacles.size(); index++)
            {
                if (obstacles[index].getBoundaryPolygon().containsPoint(p))
                {
                    unblocked_grid[col][row] = false;
                }
            }
        }
    }
}

// A Utility Function to check whether given GridCell (row, col)
// is a valid GridCell or not.
bool ThetaStarPathPlanner::isValid(int row, int col)
{
    // Returns true if row number and column number
    // is in range
    return (row >= 0) && (row < numRows) && (col >= 0) && (col < numCols);
}

// A Utility Function to check whether the given GridCell is
// blocked or not
bool ThetaStarPathPlanner::isUnBlocked(int row, int col)
{
    return unblocked_grid[row][col];
}

// A Utility Function to check whether destination GridCell has
// been reached or not
bool ThetaStarPathPlanner::isDestination(int row, int col, CellCoordinate dest)
{
    return (row == dest.first && col == dest.second);
}

// A Utility Function to calculate the 'h' heuristics.
double ThetaStarPathPlanner::calculateHValue(int row, int col, CellCoordinate dest)
{
    // Return using the distance formula
    return ((double)sqrt((row - dest.first) * (row - dest.first) +
                         (col - dest.second) * (col - dest.second)));
}

// A Utility Function to check for line of sight
bool ThetaStarPathPlanner::lineOfSight(int curr_parent_i, int curr_parent_j,
                                       CellCoordinate new_pair)
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

// A Utility Function to trace the path from the source
// to destination
std::vector<Point> ThetaStarPathPlanner::tracePath(CellCoordinate dest)
{
    int row                        = dest.first;
    int col                        = dest.second;
    std::vector<Point> path_points = std::vector<Point>();

    std::stack<CellCoordinate> Path;

    while (!(cellDetails[row][col].parent_i_ == row &&
             cellDetails[row][col].parent_j_ == col))
    {
        Path.push(std::make_pair(row, col));
        int temp_row = cellDetails[row][col].parent_i_;
        int temp_col = cellDetails[row][col].parent_j_;
        row          = temp_row;
        col          = temp_col;
    }

    Path.push(std::make_pair(row, col));
    while (!Path.empty())
    {
        CellCoordinate p = Path.top();
        Path.pop();
        path_points.push_back(
            Point(p.first * GRID_DIVISION_IN_METERS - field_.length() / 2,
                  p.second * GRID_DIVISION_IN_METERS - field_.width() / 2));
    }

    return path_points;
}

// returns true if path found
bool ThetaStarPathPlanner::updateVertex(CellCoordinate pCurr, CellCoordinate pNew,
                                        CellCoordinate dest, double currToNextNodeDist)
{
    // To store the 'g', 'h' and 'f' of the 8 successors
    double gNew, hNew, fNew;

    // Only process this GridCell if this is a valid one
    if (isValid(pNew.first, pNew.second) == true)
    {
        // If the destination GridCell is the same as the
        // current successor
        if (isDestination(pNew.first, pNew.second, dest) == true)
        {
            // Set the Parent of the destination GridCell
            cellDetails[pNew.first][pNew.second].parent_i_ = pCurr.first;
            cellDetails[pNew.first][pNew.second].parent_j_ = pCurr.second;
            return true;
        }
        // If the successor is already on the closed
        // list or if it is blocked, then ignore it.
        // Else do the following
        else if (closedList[pNew.first][pNew.second] == false &&
                 isUnBlocked(pNew.first, pNew.second) == true)
        {
            int parent_i = cellDetails[pCurr.first][pCurr.second].parent_i_;
            int parent_j = cellDetails[pCurr.first][pCurr.second].parent_j_;
            if (lineOfSight(parent_i, parent_j, pNew))
            {
                gNew = cellDetails[parent_i][parent_j].g_ +
                       calculateHValue(parent_i, parent_j, pNew);
                hNew = calculateHValue(pNew.first, pNew.second, dest);
                fNew = gNew + hNew;

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
                    openList.insert(
                        std::make_pair(fNew, std::make_pair(pNew.first, pNew.second)));

                    // Update the details of this GridCell
                    cellDetails[pNew.first][pNew.second].f_        = fNew;
                    cellDetails[pNew.first][pNew.second].g_        = gNew;
                    cellDetails[pNew.first][pNew.second].h_        = hNew;
                    cellDetails[pNew.first][pNew.second].parent_i_ = parent_i;
                    cellDetails[pNew.first][pNew.second].parent_j_ = parent_j;
                }
            }
            else
            {
                gNew = cellDetails[pCurr.first][pCurr.second].g_ + currToNextNodeDist;
                hNew = calculateHValue(pNew.first, pNew.second, dest);
                fNew = gNew + hNew;

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
                    openList.insert(
                        std::make_pair(fNew, std::make_pair(pNew.first, pNew.second)));

                    // Update the details of this GridCell
                    cellDetails[pNew.first][pNew.second].f_        = fNew;
                    cellDetails[pNew.first][pNew.second].g_        = gNew;
                    cellDetails[pNew.first][pNew.second].h_        = hNew;
                    cellDetails[pNew.first][pNew.second].parent_i_ = pCurr.first;
                    cellDetails[pNew.first][pNew.second].parent_j_ = pCurr.second;
                }
            }
        }
    }
    return false;
}

// A Function to find the shortest path between
// a given source GridCell to a destination GridCell according
// to A* Search Algorithm
std::optional<std::vector<Point>> ThetaStarPathPlanner::findPath(const Point &start,
                                                                 const Point &destination)
{
    bool blocked_dest = false;
    CellCoordinate src, dest;
    src =
        std::make_pair((int)((start.x() + field_.length() / 2) / GRID_DIVISION_IN_METERS),
                       (int)((start.y() + field_.width() / 2) / GRID_DIVISION_IN_METERS));
    dest = std::make_pair(
        (int)((destination.x() + field_.length() / 2) / GRID_DIVISION_IN_METERS),
        (int)((destination.y() + field_.width() / 2) / GRID_DIVISION_IN_METERS));

    // If the source is out of range
    if (isValid(src.first, src.second) == false)
    {
        return std::nullopt;
    }

    // If the destination is out of range
    if (isValid(dest.first, dest.second) == false)
    {
        return std::nullopt;
    }

    // The source is blocked
    if (isUnBlocked(src.first, src.second) == false)
    {
        src = findClosestUnblockedCell(src);
    }

    // The destination is blocked
    if (isUnBlocked(dest.first, dest.second) == false)
    {
        dest         = findClosestUnblockedCell(dest);
        blocked_dest = true;
    }

    // If the destination GridCell is the same as source GridCell
    if (isDestination(src.first, src.second, dest) == true)
    {
        return std::nullopt;
    }

    closedList =
        std::vector<std::vector<bool>>(numRows, std::vector<bool>(numCols, false));

    cellDetails = std::vector<std::vector<GridCell>>(
        numRows, std::vector<GridCell>(numCols, ThetaStarPathPlanner::GridCell(
                                                    -1, -1, DBL_MAX, DBL_MAX, DBL_MAX)));

    int i, j;

    // Initialising the parameters of the starting node
    i = src.first, j = src.second;
    cellDetails[i][j].f_        = 0.0;
    cellDetails[i][j].g_        = 0.0;
    cellDetails[i][j].h_        = 0.0;
    cellDetails[i][j].parent_i_ = i;
    cellDetails[i][j].parent_j_ = j;

    // Put the starting GridCell on the open list and set its
    // 'f' as 0
    openList.insert(std::make_pair(0.0, std::make_pair(i, j)));

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

                N.W N N.E
                \ | /
                \ | /
                W----Cell----E
                    / | \
                / | \
                S.W S S.E

            Cell-->Popped Cell (i, j)
            N --> North	 (i-1, j)
            S --> South	 (i+1, j)
            E --> East	 (i, j+1)
            W --> West		 (i, j-1)
            N.E--> North-East (i-1, j+1)
            N.W--> North-West (i-1, j-1)
            S.E--> South-East (i+1, j+1)
            S.W--> South-West (i+1, j-1)*/

        // To store the 'g', 'h' and 'f' of the 8 successors
        double gNew, hNew, fNew;
        CellCoordinate pCurr, pNew;
        pCurr = std::make_pair(i, j);

        //----------- 1st Successor (North) ------------

        pNew = std::make_pair(i - 1, j);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.0))
        {
            break;
        }

        //----------- 2nd Successor (South) ------------

        pNew = std::make_pair(i + 1, j);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.0))
        {
            break;
        }

        //----------- 3rd Successor (East) ------------

        pNew = std::make_pair(i, j + 1);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.0))
        {
            break;
        }

        //----------- 4th Successor (West) ------------


        pNew = std::make_pair(i, j - 1);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.0))
        {
            break;
        }

        //----------- 5th Successor (North-East) ------------

        pNew = std::make_pair(i - 1, j + 1);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.414))
        {
            break;
        }

        //----------- 6th Successor (North-West) ------------

        pNew = std::make_pair(i - 1, j - 1);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.414))
        {
            break;
        }

        //----------- 7th Successor (South-East) ------------

        pNew = std::make_pair(i + 1, j + 1);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.414))
        {
            break;
        }

        //----------- 8th Successor (South-West) ------------

        pNew = std::make_pair(i + 1, j - 1);
        if (foundDest = updateVertex(pCurr, pNew, dest, 1.414))
        {
            break;
        }
    }

    // When the destination GridCell is not found and the open
    // list is empty, then we conclude that we failed to
    // reach the destiantion GridCell. This may happen when the
    // there is no way to destination GridCell (due to blockages)
    if (foundDest == false)
    {
        return std::nullopt;
    }

    auto path_points = tracePath(dest);

    if (!blocked_dest)
    {
        // replace destination with actual destination
        // Note that this isn't needed for start since we don't care about that
        path_points.pop_back();
        path_points.push_back(destination);
    }
    return path_points;
}

// spiral out from currCell looking for unblocked cells
// this should be good enough
ThetaStarPathPlanner::CellCoordinate ThetaStarPathPlanner::findClosestUnblockedCell(
    CellCoordinate currCell)
{
    int i = currCell.first;
    int j = currCell.second;
    unsigned nextIndex, currIndex = 3;
    int nextIncrement[4] = {1, 0, -1, 0};
    for (int depth = 1; depth < numRows; depth++)
    {
        nextIndex = (currIndex + 1) % 4;
        i += nextIncrement[nextIndex] * depth;
        j += nextIncrement[currIndex] * depth;
        if (isValid(i, j) && isUnBlocked(i, j))
        {
            return std::make_pair(i, j);
        }
        currIndex = nextIndex;

        nextIndex = (currIndex + 1) % 4;
        i += nextIncrement[nextIndex] * depth;
        j += nextIncrement[currIndex] * depth;
        if (isValid(i, j) && isUnBlocked(i, j))
        {
            return std::make_pair(i, j);
        }
        currIndex = nextIndex;
    }

    return currCell;
}

//==========================//
std::optional<std::vector<Point>> ThetaStarPathPlanner::findPath(
    const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
    const ViolationFunction &violation_function)
{
    if (true)
    {
        return std::make_optional<std::vector<Point>>({start, dest});
    }
    else
    {
        return std::nullopt;
    }
}
