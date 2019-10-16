#pragma once
#include <bits/stdc++.h>
#include <unistd.h>

#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * ThetaStarPathPlanner uses the theta * algorithm to implement
 * the PathPlanner interface.
 * https://www.geeksforgeeks.org/a-search-algorithm/
 * http://aigamedev.com/open/tutorial/theta-star-any-angle-paths/
 */

class ThetaStarPathPlanner : public PathPlanner
{
   public:
    /**
     * Returns a path that is an optimized path between start and destination.
     *
     * @param start start point
     * @param destination destination point
     * @param navigableArea Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     *
     * @return a vector of points that is the optimal path avoiding obstacles
     * 		if no valid path then return empty vector
     */
    Path findPath(const Point &start, const Point &destination,
                  const Rectangle &navigableArea,
                  const std::vector<Obstacle> &obstacles) override;

   private:
    typedef std::pair<int, int> CellCoordinate;
    typedef std::pair<double, CellCoordinate> OpenListCell;

    class GridCell
    {
       public:
        GridCell(int parent_i, int parent_j, double f, double g, double h)
            : parent_i_(parent_i), parent_j_(parent_j), f_(f), g_(g), h_(h)
        {
        }

        int parent_i_, parent_j_;
        double f_, g_, h_;
    };

    /**
     * Returns if a cell is within bounds of grid
     * @param row y position of cell
     * @param col x position of cell
     *
     * @return true if cell is valid
     */
    bool isValid(int row, int col);

    /**
     * Returns if a cell is unblocked
     * @param row y position of cell
     * @param col x position of cell
     *
     * @return true if cell is unblocked
     */
    bool isUnBlocked(int row, int col);

    /**
     * Returns if a cell is the destination
     * @param row y position of cell
     * @param col x position of cell
     * @param dest destination cell
     *
     * @return true if cell is the destination
     */
    bool isDestination(int row, int col, CellCoordinate dest);

    /**
     * Returns heuristic value of a cell
     * This is currently the Euclidean distance to the destination
     * @param row y position of cell
     * @param col x position of cell
     * @param dest destination cell
     *
     * @return Euclidean distance to dest
     */
    double calculateHValue(int row, int col, CellCoordinate dest);

    /**
     * Traces a path from the destination back to the start
     * and populates a vector of points with that path
     * @param dest destination cell
     *
     * @return vector of points with the path from start to dest
     */
    std::vector<Point> tracePath(CellCoordinate dest);

    /**
     * Updates the new node's fields based on the current node, destination
     * and the distance to the next node
     * and checks if destination is reached
     * @param pCurr                 current cell
     * @param pNext                 next cell to be updated
     * @param dest                  destination cell
     * @param currToNextNodeDist    Euclidean distance between pCurr and pNext
     *
     * @return                      true if pNew is destination
     */
    bool updateVertex(CellCoordinate pCurr, CellCoordinate pNew, CellCoordinate dest,
                      double currToNextNodeDist);

    /**
     * Checks for line of sight between parent cell and new cell
     * @param curr_parent_i         parent cell's x coordinate
     * @param curr_parent_j         parent cell's y coordinate
     * @param new_pair              cell to check line of sight to
     *
     * @return                      true if line of sight from parent to new cell
     */
    bool lineOfSight(int curr_parent_i, int curr_parent_j, CellCoordinate new_pair);

    /**
     * Finds closest unblocked cell to currCell
     * @param currCell  current cell
     *
     * @return          closest unblocked cell to currCell
     *                  if none found, return nullopt
     */
    std::optional<CellCoordinate> findClosestUnblockedCell(CellCoordinate currCell);

    /**
     * Finds closest valid point that's not in an obstacle to p
     * @param p     a given point
     *
     * @return          closest free point to currCell
     *                  if not blocked then return p
     */
    Point findClosestFreePoint(Point p);

    /**
     * Checks if a point is valid and doesn't exist in any obstacles
     * @param p     a given point
     *
     * @return      if p is valid and isn't in an obstacle
     * */
    bool isValidAndFreeOfObstacles(Point p);

    /**
     * Converts a cell in grid to a point on field
     *
     * @param row row of cell
     * @param col col of cell
     *
     * @return Point on field
     */
    Point convertCellToPoint(int row, int col);

    /**
     * Converts a point on field to a cell in grid
     *
     * @param p point on field
     *
     * @return cell in grid
     */
    CellCoordinate convertPointToCell(Point p);

    // if close to destination then return no path
    static constexpr double CLOSE_TO_DEST_THRESHOLD = 0.01;  // in metres

    // increase in threshold to reduce oscillation
    static constexpr int BLOCKED_DESINATION_OSCILLATION_MITIGATION =
        2;  // multiples of CLOSE_TO_DEST_THRESHOLD to ignore to control oscillation

    // resolution for searching for unblocked point around a blocked destination
    static constexpr double BLOCKED_DESTINATION_SEARCH_RESOLUTION =
        50.0;  // number of fractions to divide 1m

    // only change this value
    static constexpr int GRID_DIVISION_FACTOR = 1;  // the n in the O(n^2) algorithm :p
    // don't change this calculation
    const double SIZE_OF_GRID_CELL_IN_METERS =
        (ROBOT_MAX_RADIUS_METERS / GRID_DIVISION_FACTOR);

    std::vector<Obstacle> obstacles_;
    int numRows;
    int numCols;
    double fieldXLength;
    double fieldYLength;
    double fieldXHalfLength;
    double fieldYHalfLength;

    /*
    Create an open list having information as-
    <f, <i, j>>
    where f = g + h,
    and i, j are the row and column index of that GridCell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implenented as a set of pair of pair.*/
    std::set<OpenListCell> openList;

    // Create a closed list and initialise it to false which means
    // that no GridCell has been included yet
    // This closed list is implemented as a boolean 2D array
    std::vector<std::vector<bool>> closedList;

    // Declare a 2D array of structure to hold the details
    // of that GridCell
    std::vector<std::vector<GridCell>> cellDetails;


    // Description of the Grid-
    // true --> The cell is not blocked
    // false --> The cell is blocked
    // We update this as we go to avoid updating cells we don't use
    std::map<std::pair<int, int>, bool> unblocked_grid;
};
