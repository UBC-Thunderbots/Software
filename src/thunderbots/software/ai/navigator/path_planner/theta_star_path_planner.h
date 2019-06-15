#pragma once
#include <bits/stdc++.h>
#include <unistd.h>

#include "ai/navigator/path_planner/path_planner.h"
#include "ai/world/ball.h"
#include "ai/world/field.h"

#define GRID_DIVISION_IN_METERS (ROBOT_MAX_RADIUS_METERS)

/**
 * ThetaStarPathPlanner uses the theta * algorithm to implement
 * the PathPlanner interface.
 * https://www.geeksforgeeks.org/a-search-algorithm/
 * http://aigamedev.com/open/tutorial/theta-star-any-angle-paths/
 */

class ThetaStarPathPlanner : public PathPlanner
{
    // Creating a shortcut for int, int pair type
    typedef std::pair<int, int> CellCoordinate;

    // Creating a shortcut for std::pair<int, std::pair<int, int>> type
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

   public:
    /**
     * Constructs a theta star path planner
     * @param field field
     * @param ball ball
     * @param obstacles obstacles to avoid
     */
    explicit ThetaStarPathPlanner(Field field, Ball ball,
                                  const std::vector<Obstacle> &obstacles);

    /**
     * Returns a path that is an optimized path between start and dest.
     * @param start start point
     * @param dest destination point
     * @return a vector that is the optimal path avoiding obstacles
     * 		if no path void then return std::nullopt
     */
    std::optional<std::vector<Point>> findPath(const Point &start,
                                               const Point &dest) override;

    /**
     * Returns a path that is a straight line between start and dest.
     * @param start start point
     * @param dest destination point
     * @param obstacles obstacles to avoid
     * @param violation_function unused parameter
     * @return a vector that is the optimal path avoiding obstacles
     * 		if no path void then return std::nullopt
     */
    std::optional<std::vector<Point>> findPath(
        const Point &start, const Point &dest, const std::vector<Obstacle> &obstacles,
        const ViolationFunction &violation_function) override;

   private:
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


    bool isValid(int row, int col);
    bool isUnBlocked(int row, int col);
    bool isDestination(int row, int col, CellCoordinate dest);
    double calculateHValue(int row, int col, CellCoordinate dest);
    std::vector<Point> tracePath(CellCoordinate dest);
    bool updateVertex(CellCoordinate pCurr, CellCoordinate pNew, CellCoordinate dest,
                      double currToNextNodeDist);
    bool lineOfSight(int curr_parent_i, int curr_parent_j, CellCoordinate new_pair);


    Field field_;
    Ball ball_;
    std::vector<Obstacle> obstacles_;
    int numRows;
    int numCols;
};
