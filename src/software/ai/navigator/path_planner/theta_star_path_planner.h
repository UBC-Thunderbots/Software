#pragma once

#include <cassert>
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
    class Coordinate
    {
       public:
        Coordinate(unsigned int row, unsigned int col)
            : row_(row), col_(col), internal_comparison_key_(computeComparisonKey(*this))
        {
        }

        Coordinate() : row_(0), col_(0) {}

        unsigned int row(void) const
        {
            return row_;
        }

        unsigned int col(void) const
        {
            return col_;
        }

        unsigned int internalComparisonKey(void) const
        {
            return internal_comparison_key_;
        }
        bool operator<(const Coordinate &other) const
        {
            return internal_comparison_key_ < other.internal_comparison_key_;
        }

        bool operator==(const Coordinate &other) const
        {
            return internal_comparison_key_ == other.internal_comparison_key_;
        }

       private:
        /**
         * Calculate the key of this coordinate based on row and col for comparison. row
         * and col are smaller than 1<<16 (65536) row value will occupy low 16 bits and
         * col values will occupy high 16 bits in key
         *
         * @param coord the reference to this coordinate itself
         * @return the key value in unsigned int
         */

        static unsigned int computeComparisonKey(const Coordinate &coord)
        {
            return coord.row() + coord.col() * (1 << 16);
        }

        // Each point on the field is discretized and fit into a 2d grid on the navigable
        // area. row_ is number of the cell that the coordinate is located in the x
        // direction. col_ is the number of the cell that the coordinate is in the y
        // direction. internal_comparison_key_ is uniquely assigned to each cell in the
        // grid for the purpose of comparison
        unsigned int row_;
        unsigned int col_;
        unsigned int internal_comparison_key_;
    };

    class CoordinatePair
    {
       public:
        CoordinatePair(const Coordinate &coord1, const Coordinate &coord2)
            : coord1_(coord1),
              coord2_(coord2),
              internal_comparison_key_(computeComparisonKey(coord1, coord2))
        {
        }

        CoordinatePair() : coord1_(), coord2_() {}

        const Coordinate &firstCoordinate(void) const
        {
            return coord1_;
        }

        const Coordinate &secondCoordinate(void) const
        {
            return coord2_;
        }

        bool operator<(const CoordinatePair &other) const
        {
            return internal_comparison_key_ < other.internal_comparison_key_;
        }

       private:
        /**
         * Calculate the key value given a pair of coordinates. The key of the coordinate
         * with smaller row value, or smaller col value when row values are equal will
         * occupy the low 32 bits in the key for this CoordinatePair. The key value of the
         * other coordinate will occupy the high 32 bits in the key for this
         * CoordinatePair.
         *
         * @param coord1 the first coordinate
         * @param coord2 the second coordinate
         * @return the key value in unsigned long
         */
        static unsigned long computeComparisonKey(const Coordinate &coord1,
                                                  const Coordinate &coord2)
        {
            unsigned long key1 = coord1.internalComparisonKey();
            unsigned long key2 = coord2.internalComparisonKey();
            if (coord1.row() < coord2.row() ||
                (coord1.row() == coord2.row() && coord1.col() < coord2.col()))
            {
                return key1 + key2 * (((unsigned long)1) << 32);
            }
            else
            {
                return key2 + key1 * (((unsigned long)1) << 32);
            }
        }

        Coordinate coord1_;
        Coordinate coord2_;
        unsigned long internal_comparison_key_;
    };

    class CellHeuristic
    {
       public:
        CellHeuristic()
            : parent_(0, 0),
              start_to_end_cost_estimate_(0),
              best_path_cost_(0),
              initialized_(false)
        {
        }

        /**
         * Updates CellHeuristics internal variables
         * Once updated, a CellHeuristic is considered initialized
         *
         * @param parent parent
         * @param start_to_end_cost_estimate The start to end_cost estimate
         * @param best_path_cost best_path_cost
         */
        void update(const Coordinate &parent, double start_to_end_cost_estimate,
                    double best_path_cost)
        {
            parent_                     = parent;
            start_to_end_cost_estimate_ = start_to_end_cost_estimate;
            best_path_cost_             = best_path_cost;
            initialized_                = true;
        }

        /**
         * Checks if this is initialized
         *
         * @return true if CellHeuristic is initialized
         */
        bool isInitialized(void) const
        {
            return initialized_;
        }

        /**
         * Gets start to end cost estimate, which is the sum of the best known real path
         * cost from the cell to start plus the Euclidean distance between the cell and
         * end
         *
         * @return start_to_end_cost_estimate
         */
        double pathCostAndEndDistHeuristic() const
        {
            return start_to_end_cost_estimate_;
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
        double start_to_end_cost_estimate_;
        double best_path_cost_;
        bool initialized_;
    };

    /**
     * Returns whether or not a cell is within bounds of grid
     *
     * @param coord Coordinate to consider
     *
     * @return true if cell is navigable
     */
    bool isCoordNavigable(const Coordinate &coord) const;

    /**
     * Returns whether or not a cell is unblocked
     *
     * @param coord Coordinate to consider
     *
     * @return true if cell is unblocked
     */
    bool isUnblocked(const Coordinate &coord);

    /**
     * Computes Euclidean distance from coord1 to coord2
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
     *
     * @return true if next is the end
     */
    bool updateVertex(const Coordinate &current, const Coordinate &next,
                      const Coordinate &end);

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
     * @return true if path to end was found
     */
    bool findPathToEnd(const Coordinate &end_coord);

    /**
     * Update vertex for all neighbours (all 8 directions) of current_coord
     *
     * @param current_coord The current coordinate
     * @param end_coord end coordinates
     *
     * @return true if path to end was found among successors
     */
    bool visitNeighbours(const Coordinate &current_coord, const Coordinate &end_coord);

    /**
     * Adjusts start and end points for navigability and determines if a path cannot be
     * found
     *
     * If start or end are non-navigable, then no path exists
     *
     * If start or end are blocked by obstacle(s), then try to find a nearby point that
     * isn't blocked
     * - If a point can be found then set the endpoint to that nearest point
     * - If no nearby points are unblocked, then no path exists
     *
     * @param [in/out] start_coord source coordinate
     * @param [in/out] end_coord end coordinate
     *
     * @return true if there is no path to end
     */
    bool adjustEndPointsAndCheckForNoPath(Coordinate &start_coord, Coordinate &end_coord);

    /**
     * Resets and initializes member variables to prepare for planning a new path
     *
     * @param navigable_area Rectangle representing the navigable area
     * @param obstacles obstacles to avoid
     */
    void resetAndInitializeMemberVariables(const Rectangle &navigable_area,
                                           const std::vector<ObstaclePtr> &obstacles);

    // if close to end then return direct path to end point
    static constexpr double CLOSE_TO_END_THRESHOLD = 0.01;  // in metres

    // resolution for searching for unblocked point around a blocked end
    static constexpr double BLOCKED_END_SEARCH_RESOLUTION =
        50.0;  // number of fractions to divide 1m

    const double SIZE_OF_GRID_CELL_IN_METERS =
        ROBOT_MAX_RADIUS_METERS;  // this is the n in the O(n^2) algorithm :p

    std::vector<ObstaclePtr> obstacles;
    Point centre;
    unsigned int num_grid_rows;
    unsigned int num_grid_cols;
    double max_navigable_x_coord;
    double max_navigable_y_coord;

    // open_list represents Coordinates that we'd like to visit. Elements are pairs of
    // start_to_end_cost_estimate and Coordinate, so the set is implicitly ordered by
    // start_to_end_cost_estimate (and then by Coordinate to break ties). This ensures
    // that open_list.begin() is the Coordinate with the lowest start_to_end_cost_estimate
    std::set<std::pair<double, Coordinate>> open_list;

    // closed_list represent coords we've already visited so
    // it contains coords for which we calculated the CellHeuristic
    std::set<Coordinate> closed_list;

    // Declare a 2D array of structure to hold the details of that CellHeuristic
    std::vector<std::vector<CellHeuristic>> cell_heuristics;

    // The following data structures improve performance by caching the results of
    // isUnblocked and lineOfSight.
    // Description of the Grid-
    // unblocked_grid is indexed with coordinate
    // true --> The cell is not blocked
    // false --> The cell is blocked
    // We update this as we go to avoid updating cells we don't use
    std::map<Coordinate, bool> unblocked_grid;
    // Cache of line of sight that maps a pair of
    // coordinates to whether those two Coordinates have line of sight between them
    std::map<CoordinatePair, bool> line_of_sight_cache;
};
