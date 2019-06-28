#pragma once
#include <bits/stdc++.h>
#include <unistd.h>

#include "ai/navigator/path_planner/path_planner.h"
#include "ai/world/ball.h"
#include "ai/world/field.h"

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
     * Constructs a theta star path planner
     * @param field field
     * @param ball ball
     * @param obstacles obstacles to avoid
     */
    explicit ThetaStarPathPlanner(Field field, const std::vector<Obstacle> &obstacles);

    /**
     * Returns a path that is an optimized path between start and dest.
     * @param start start point
     * @param dest destination point
     * @return a vector that is the optimal path avoiding obstacles
     * 		if no valid path then return std::nullopt
     */
    std::optional<std::vector<Point>> findPath(const Point &start,
                                               const Point &dest) override;

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


    /* Description of the Grid-
    true --> The cell is not blocked
    false --> The cell is blocked */
    std::vector<std::vector<bool>> unblocked_grid;


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

    static constexpr double CLOSE_TO_DEST_THRESHOLD = 0.01;
    static constexpr double RESOLUTION_FACTOR       = 100.0;

    // only change this value
    static constexpr int GRID_DIVISION_FACTOR = 1;  // the n in the O(n^2) algorithm :p
    // don't change this calculation
    const double SIZE_OF_GRID_CELL_IN_METERS =
        (ROBOT_MAX_RADIUS_METERS / GRID_DIVISION_FACTOR);

    Field field_;
    std::vector<Obstacle> obstacles_;
    int numRows;
    int numCols;
};
