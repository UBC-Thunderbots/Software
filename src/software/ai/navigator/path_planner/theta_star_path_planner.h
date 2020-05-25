#pragma once

#include <unistd.h>

#include <map>
#include <set>

#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * ThetaStarPathPlanner uses the theta * algorithm to implement
 * the PathPlanner interface.
 * It is a grid-based graph algorithm, but it allows any two nodes to be connected to each
 * other, as long as there's line of sight. It will optimize for the shortest total path
 * length. Theta Star (Theta*) is closely related to A Star (A*) (see
 * https://www.geeksforgeeks.org/a-search-algorithm/), but it will implicitly smooth paths
 * as it explores the graph. Read
 * https://web.archive.org/web/20190218161704/http://aigamedev.com/open/tutorial/theta-star-any-angle-paths/
 * for an explanation of how that works, including pseudocode and diagrams.
 */

class ThetaStarPathPlanner : public PathPlanner
{
   public:
    ThetaStarPathPlanner();

    /**
     * Returns a path that is an optimized path between start and destination.
     *
     * @param start start point
     * @param destination destination point
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     *
     * @return a vector of points that is the optimal path avoiding obstacles
     *         if no valid path then return empty vector
     */
    std::optional<Path> findPath(const Point &start, const Point &destination,
                                 const Rectangle &navigable_area,
                                 const std::vector<ObstaclePtr> &obstacles) override;

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

    class CellHeuristic
    {
       public:
        CellHeuristic() : parent_(0, 0), f_(0), g_(0), initialized_(false) {}

        /**
         * Updates CellHeuristics internal variables
         * Once updated, a CellHeuristic is considered intialized
         *
         * @param parent parent
         * @param f f value
         * @param g g value
         */
        void update(Coordinate parent, double f, double g)
        {
            parent_      = parent;
            f_           = f;
            g_           = g;
            initialized_ = true;
        }

        /**
         * Checks if this is initialized
         *
         * @return if CellHeuristic is initialized
         */
        bool isInitialized(void) const
        {
            return initialized_;
        }

        /**
         * Gets f value
         *
         * @return f value
         */
        double f() const
        {
            return f_;
        }

        /**
         * Gets g value
         *
         * @return g value
         */
        double g() const
        {
            return g_;
        }

        /**
         * Gets parent Coordinate
         *
         * @return parent
         */
        Coordinate parent() const
        {
            return parent_;
        }

       private:
        Coordinate parent_;
        double f_, g_;
        bool initialized_;
    };

    /**
     * Returns if a cell is within bounds of grid
     *
     * @param test_coord Coordinate to consider
     *
     * @return true if cell is valid
     */
    bool isCoordValid(Coordinate test_coord);

    /**
     * Returns if a cell is unblocked
     *
     * @param test_coord Coordinate to consider
     *
     * @return true if cell is unblocked
     */
    bool isUnBlocked(Coordinate test_coord);

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
     *
     * @param current_parent        parent cell
     * @param new_pair              cell to check line of sight to
     *
     * @return                      true if line of sight from parent to new cell
     */
    bool hasLineOfSight(Coordinate current_parent, Coordinate new_pair);

    /**
     * Finds closest unblocked cell to current_cell
     *
     * @param current_cell  current cell
     *
     * @return          closest unblocked cell to current_cell
     *                  if none found, return nullopt
     */
    std::optional<Coordinate> findClosestUnblockedCell(Coordinate current_cell);

    /**
     * Finds closest valid point that's not in an obstacle to p
     *
     * @param p     a given point
     *
     * @return          closest free point to currCell
     *                  if not blocked then return p
     */
    Point findClosestFreePoint(Point p);

    /**
     * Checks if a point is valid and doesn't exist in any obstacles
     *
     * @param p     a given point
     *
     * @return      if p is valid and isn't in an obstacle
     * */
    bool isPointValidAndFreeOfObstacles(Point p);

    /**
     * Checks if a point is valid
     *
     * @param p     a given point
     *
     * @return      if p is valid
     * */
    bool isPointValid(Point p);

    /**
     * Converts a cell in grid to a point on navigable area
     *
     * @param coord Coordinate to convert
     *
     * @return Point on navigable area
     */
    Point coordinateToPoint(Coordinate coord);

    /**
     * Converts a point on navigable area to a cell in grid
     *
     * @param p point on navigable area
     *
     * @return cell in grid
     */
    Coordinate pointToCoordinate(Point p);

    /**
     * Try to find a path to destination and leave
     * trail markers along the way
     *
     * @param dest_coord destination coordinates
     *
     * @return if path to destination was found
     */
    bool findPathToDestination(Coordinate dest_coord);

    /**
     * Check for invalid or blocked src_coord and dest_coord and
     * adjust parameters accordingly
     *
     * @param src_coord source coordinate
     * @param dest_coord destination coordinate
     *
     * @return true if there is no path to destination
     */
    bool checkForInvalidOrBlockedCases(Coordinate &src_coord, Coordinate &dest_coord);

    /**
     * Check if start to destination is at least a grid cell away
     *
     * @param start start point
     * @param destination destination point
     *
     * @return true start to destination is within threshold
     */
    bool isStartToDestinationWithinThreshold(const Point &start,
                                             const Point &destination);

    /**
     * Check if start to closest destination is at least a grid cell away
     *
     * @param start start point
     * @param closest_destination closest destination point
     *
     * @return true start to closest destination is within threshold
     */
    bool isStartToClosestDestinationWithinThreshold(const Point &start,
                                                    const Point &closest_destination);

    /**
     * Resets and initializes member variables to prepare for planning a new path
     *
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     */
    void resetAndInitializeMemberVariables(const Rectangle &navigable_area,
                                           const std::vector<ObstaclePtr> &obstacles);

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

    std::vector<ObstaclePtr> obstacles;
    int num_grid_rows;
    int num_grid_cols;
    double max_navigable_x_coord;
    double max_navigable_y_coord;

    // open_list represents Coordinates that we'd like to visit
    std::set<Coordinate> open_list;

    // closed_list represent coords we've already visited so
    // it contains coords for which we calculated the CellHeuristic
    std::set<Coordinate> closed_list;

    // Declare a 2D array of structure to hold the details
    // of that CellHeuristic
    std::vector<std::vector<CellHeuristic>> cell_heuristics;


    // Description of the Grid-
    // true --> The cell is not blocked
    // false --> The cell is blocked
    // We update this as we go to avoid updating cells we don't use
    std::map<Coordinate, bool> unblocked_grid;
};
