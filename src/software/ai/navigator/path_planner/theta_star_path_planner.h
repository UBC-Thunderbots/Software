#pragma once

#include <map>
#include <set>

#include "software/ai/navigator/path_planner/path_planner.h"

/**
 * ThetaStarPathPlanner uses the theta * algorithm to implement
 * the PathPlanner interface.
 * It is a grid-based graph algorithm, but it allows any two cells to be connected to each
 * other, as long as there's line of sight. It will optimize for the shortest total path
 * length. Theta Star (Theta*) is closely related to A Star (A*) (see
 * https://www.geeksforgeeks.org/a-search-algorithm/), but it will implicitly smooth paths
 * as it explores the graph.
 *
 * Read
 * https://web.archive.org/web/20190218161704/http://aigamedev.com/open/tutorial/theta-star-any-angle-paths/
 * for an explanation of how that works, including pseudocode and diagrams.
 */

class ThetaStarPathPlanner : public PathPlanner
{
   public:
    ThetaStarPathPlanner();

    /**
     * Returns a path that is an optimized path between start and end.
     *
     * @param start start point
     * @param end end point
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     *
     * @return a vector of points that is the optimal path avoiding obstacles
     *         if no valid path then return empty vector
     */
    std::optional<Path> findPath(const Point &start, const Point &end,
                                 const Rectangle &navigable_area,
                                 const std::vector<ObstaclePtr> &obstacles) override;

   private:
    class Coordinate : public std::pair<unsigned int, unsigned int>
    {
       public:
        Coordinate(unsigned int row, unsigned int col)
            : std::pair<unsigned int, unsigned int>(row, col)
        {
        }

        Coordinate() : std::pair<unsigned int, unsigned int>() {}

        unsigned int row(void) const
        {
            return this->first;
        }

        unsigned int col(void) const
        {
            return this->second;
        }
    };

    class CellHeuristic
    {
       public:
        CellHeuristic()
            : parent_(0, 0),
              path_cost_and_end_dist_heuristic_(0),
              best_path_cost_(0),
              initialized_(false)
        {
        }

        /**
         * Updates CellHeuristics internal variables
         * Once updated, a CellHeuristic is considered intialized
         *
         * @param parent parent
         * @param path_cost_and_end_dist_heuristic The path cost and end dist heuristic
         * @param best_path_cost best_path_cost
         */
        void update(Coordinate parent, double path_cost_and_end_dist_heuristic,
                    double best_path_cost)
        {
            parent_                           = parent;
            path_cost_and_end_dist_heuristic_ = path_cost_and_end_dist_heuristic;
            best_path_cost_                   = best_path_cost;
            initialized_                      = true;
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
         * Gets path cost and end dist heuristic, which is the sum of the best path cost
         * from the cell to end plus the Euclidean distance between the cell and end
         *
         * @return path cost and end dist heuristic
         */
        double pathCostAndEndDistHeuristic() const
        {
            return path_cost_and_end_dist_heuristic_;
        }

        /**
         * Gets best_path_cost, the best path cost from the cell to end
         *
         * @return the best path cost
         */
        double bestPathCost() const
        {
            return best_path_cost_;
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
        double path_cost_and_end_dist_heuristic_;
        double best_path_cost_;
        bool initialized_;
    };

    /**
     * Returns if a cell is within bounds of grid
     *
     * @param coord Coordinate to consider
     *
     * @return true if cell is navigable
     */
    bool isCoordNavigable(const Coordinate &coord) const;

    /**
     * Returns if a cell is unblocked
     *
     * @param coord Coordinate to consider
     *
     * @return true if cell is unblocked
     */
    bool isUnblocked(const Coordinate &coord);

    /**
     * Computes distance from coord1 to coord2
     *
     * @param coord1 The first Coordinate
     * @param coord2 The second Coordinate
     *
     * @return distance between coord1 to coord2
     */
    double coordDistance(const Coordinate &coord1, const Coordinate &coord2) const;

    /**
     * Traces a path from the end back to the start
     * and populates a vector of points with that path
     *
     * @param end end cell
     *
     * @return vector of points with the path from start to end
     */
    std::vector<Point> tracePath(const Coordinate &end) const;

    /**
     * Updates the next cell's fields based on the current cell, end and the
     * distance to the next cell and checks if end is reached
     *
     * @param current The current cell
     * @param next    The next cell to be updated
     * @param end     The end cell
     * @param marginal_dist The distance between current and next
     *
     * @return true if next is the end
     */
    bool updateVertex(const Coordinate &current, const Coordinate &next,
                      const Coordinate &end, double marginal_dist);

    /**
     * Checks for line of sight between Coordinates
     *
     * @param coord1 The first Coordinate
     * @param coord2 The second Coordinate
     *
     * @return true if line of sight from coord1 to coord2
     */
    bool lineOfSight(const Coordinate &coord1, const Coordinate &coord2);

    /**
     * Finds closest unblocked cell to current_cell
     *
     * @param current_cell  current cell
     *
     * @return          closest unblocked cell to current_cell
     *                  if none found, return nullopt
     */
    std::optional<Coordinate> findClosestUnblockedCell(const Coordinate &current_cell);

    /**
     * Finds closest navigable point that's not in an obstacle to p
     *
     * @param p     a given point
     *
     * @return          closest free point to currCell
     *                  if not blocked then return p
     */
    Point findClosestFreePoint(const Point &p);

    /**
     * Checks if a point is navigable and doesn't exist in any obstacles
     *
     * @param p     a given point
     *
     * @return      if p is navigable and isn't in an obstacle
     * */
    bool isPointNavigableAndFreeOfObstacles(const Point &p);

    /**
     * Checks if a point is navigable
     *
     * @param p     a given point
     *
     * @return      if p is navigable
     * */
    bool isPointNavigable(const Point &p) const;

    /**
     * Converts a cell in grid to a point on navigable area
     *
     * @param coord Coordinate to convert
     *
     * @return Point on navigable area
     */
    Point convertCoordToPoint(const Coordinate &coord) const;

    /**
     * Converts a point on navigable area to a cell in grid
     *
     * @param p point on navigable area
     *
     * @return cell in grid
     */
    Coordinate convertPointToCoord(const Point &p) const;

    /**
     * Try to find a path to end and leave
     * trail markers along the way
     *
     * @param end_coord end coordinates
     *
     * @return if path to end was found
     */
    bool findPathToEnd(const Coordinate &end_coord);

    /**
     * Update vertex for all successors of current_coord
     *
     * @param current_coord The current coordinate
     * @param end_coord end coordinates
     *
     * @return if path to end was found among successors
     */
    bool visitSuccessors(const Coordinate &current_coord, const Coordinate &end_coord);

    /**
     * Check for non-navigable or blocked start_coord and end_coord and
     * adjust parameters accordingly
     *
     * @param [in/out] start_coord source coordinate
     * @param [in/out] end_coord end coordinate
     *
     * @return true if there is no path to end
     */
    bool nonNavigableOrBlockedCases(Coordinate &start_coord, Coordinate &end_coord);

    /**
     * Check if start to end is at least a grid cell away
     *
     * @param start start point
     * @param end end point
     *
     * @return true start to end is within threshold
     */
    bool isStartToEndWithinThreshold(const Point &start, const Point &end) const;

    /**
     * Check if start to closest end is at least a grid cell away
     *
     * @param start start point
     * @param closest_end closest end point
     *
     * @return true start to closest end is within threshold
     */
    bool isStartToClosestEndWithinThreshold(const Point &start,
                                            const Point &closest_end) const;

    /**
     * Resets and initializes member variables to prepare for planning a new path
     *
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     */
    void resetAndInitializeMemberVariables(const Rectangle &navigable_area,
                                           const std::vector<ObstaclePtr> &obstacles);

    /**
     * Returns a collision-free hash for the given Coordinate
     *
     * @param coord Coordinate to hash
     *
     * @return hash of the Coordinate
     */
    unsigned long hashCoordinate(const Coordinate &coord) const;

    /**
     * Returns a collision-free hash for the given a pair of Coordinate
     *
     * @param coord1 Coordinate 1 of pair to hash
     * @param coord2 Coordinate 2 of pair to hash
     *
     * @return hash of the pair of Coordinates
     */
    unsigned long hashCoordinatePair(
        const ThetaStarPathPlanner::Coordinate &coord1,
        const ThetaStarPathPlanner::Coordinate &coord2) const;

    // if close to end then return no path
    static constexpr double CLOSE_TO_DEST_THRESHOLD = 0.01;  // in metres

    // increase in threshold to reduce oscillation
    static constexpr unsigned int BLOCKED_DESINATION_OSCILLATION_MITIGATION =
        2;  // multiples of CLOSE_TO_DEST_THRESHOLD to ignore to control oscillation

    // resolution for searching for unblocked point around a blocked end
    static constexpr double BLOCKED_DESTINATION_SEARCH_RESOLUTION =
        50.0;  // number of fractions to divide 1m

    // only change this value
    static constexpr unsigned int GRID_DIVISION_FACTOR =
        1;  // the n in the O(n^2) algorithm :p
    // don't change this calculation
    const double SIZE_OF_GRID_CELL_IN_METERS =
        (ROBOT_MAX_RADIUS_METERS / GRID_DIVISION_FACTOR);

    std::vector<ObstaclePtr> obstacles;
    unsigned int num_grid_rows;
    unsigned int num_grid_cols;
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

    // The following data structures improve performance by caching the results of
    // isUnblocked and lineOfSight.
    // They are indexed with a hash for performance
    // Description of the Grid-
    // true --> The cell is not blocked
    // false --> The cell is blocked
    // We update this as we go to avoid updating cells we don't use
    std::map<unsigned long, bool> unblocked_grid;
    // Cache of line of sight that maps a hash of a pair of coordinates to whether those
    // two Coordinates have line of sight between them
    std::map<unsigned long, bool> line_of_sight_cache;
};
