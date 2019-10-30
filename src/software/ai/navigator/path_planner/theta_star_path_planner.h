#pragma once
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
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     *
     * @return a vector of points that is the optimal path avoiding obstacles
     * 		if no valid path then return empty vector
     */
    Path findPath(const Point &start, const Point &destination,
                  const Rectangle &navigable_area,
                  const std::vector<Obstacle> &obstacles) override;

   private:
    class Coordinate : public std::pair<int, int>
    {
       public:
        Coordinate(int row, int col) : std::pair<int, int>(row, col) {}

        Coordinate() : std::pair<int, int>() {}

        int row(void)
        {
            return this->first;
        }

        int col(void)
        {
            return this->second;
        }
    };

    class GridCell
    {
       public:
        GridCell(Coordinate parent, double f, double g, double h)
            : parent(parent), f(f), g(g), h(h)
        {
        }

        Coordinate parent;
        double f, g, h;
    };

    using OpenListCell = std::pair<double, Coordinate>;

    /**
     * Returns if a cell is within bounds of grid
     *
     * @param test_coord Coordinate to consider
     *
     * @return true if cell is valid
     */
    bool isValid(Coordinate test_coord);

    /**
     * Returns if a cell is unblocked
     *
     * @param test_coord Coordinate to consider
     *
     * @return true if cell is unblocked
     */
    bool isUnBlocked(Coordinate test_coord);

    /**
     * Returns if a cell is the destination
     *
     * @param test_coord Coordinate to consider
     * @param dest destination cell
     *
     * @return true if cell is the destination
     */
    bool isDestination(Coordinate test_coord, Coordinate dest);

    /**
     * Returns heuristic value of a cell,
     * currently the Euclidean distance to the destination
     *
     * @param test_coord Coordinate to consider
     * @param dest destination Coordinate
     *
     * @return Euclidean distance to dest
     */
    double calculateHValue(Coordinate test_coord, Coordinate dest);

    /**
     * Traces a path from the destination back to the start
     * and populates a vector of points with that path
     *
     * @param dest destination cell
     *
     * @return vector of points with the path from start to dest
     */
    std::vector<Point> tracePath(Coordinate dest);

    /**
     * Updates the new node's fields based on the current node, destination
     * and the distance to the next node
     * and checks if destination is reached
     *
     * @param current_coord         current cell
     * @param new_coord                 next cell to be updated
     * @param dest                  destination cell
     * @param curr_to_new_dist    Euclidean distance between current_coord and new_coord
     *
     * @return                      true if new_coord is destination
     */
    bool updateVertex(Coordinate current_coord, Coordinate new_coord, Coordinate dest,
                      double curr_to_new_dist);

    /**
     * Checks for line of sight between parent cell and new cell
     * @param current_parent        parent cell
     * @param new_pair              cell to check line of sight to
     *
     * @return                      true if line of sight from parent to new cell
     */
    bool lineOfSight(Coordinate current_parent, Coordinate new_pair);

    /**
     * Finds closest unblocked cell to current_cell
     * @param current_cell  current cell
     *
     * @return          closest unblocked cell to current_cell
     *                  if none found, return nullopt
     */
    std::optional<Coordinate> findClosestUnblockedCell(Coordinate current_cell);

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
     * Converts a cell in grid to a point on navigable area
     *
     * @param coord Coordinate to convert
     *
     * @return Point on navigable area
     */
    Point convertCoordinateToPoint(Coordinate coord);

    /**
     * Converts a point on navigable area to a cell in grid
     *
     * @param p point on navigable area
     *
     * @return cell in grid
     */
    Coordinate convertPointToCoordinate(Point p);

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

    std::vector<Obstacle> obstacles;
    int num_grid_rows;
    int num_grid_cols;
    double max_navigable_x_coord;
    double max_navigable_y_coord;

    /*
    Create an open list having information as-
    <f, <i, j>>
    where f = g + h,
    and i, j are the row and column index of that GridCell
    Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
    This open list is implenented as a set of pair of pair.*/
    std::set<OpenListCell> open_list;

    // Create a closed list and initialise it to false which means
    // that no GridCell has been included yet
    // This closed list is implemented as a boolean 2D array
    std::vector<std::vector<bool>> closed_list;

    // Declare a 2D array of structure to hold the details
    // of that GridCell
    std::vector<std::vector<GridCell>> cell_details;


    // Description of the Grid-
    // true --> The cell is not blocked
    // false --> The cell is blocked
    // We update this as we go to avoid updating cells we don't use
    std::map<Coordinate, bool> unblocked_grid;
};
