#pragma once

#include <boost/functional/hash/hash.hpp>
#include <queue>

#include "extlibs/enlsvg/ENLSVG.h"

#include "software/ai/navigator/obstacle/obstacle.hpp"
#include "software/geom/linear_spline2d.h"
#include "software/logger/logger.h"
#include "software/world/world.h"

using Path = std::vector<Point>;

/**
 * The Edge N-Level Sparse Visibility Graph algorithm is a fast pathfinding algorithm for
 * precomputed grids. This class is a wrapper for an implementation found in
 * software/src/extlibs/enlsvg/. This slide deck from Ohohcaketester
 * [https://github.com/Ohohcakester/Any-Angle-Pathfinding/files/1286724/ENLSVG_SoCS17.pdf]
 * and this research paper [https://aaai.org/ocs/index.php/SOCS/SOCS17/paper/view/15790]
 * provides more information.
 *
 * Since this algorithm requires precomputation to achieve fast pathfinding, obstacles are
 * passed into the constructor and is unmodifiable once it is created.
 */
class EnlsvgPathPlanner
{
   public:
    /**
     * Creates an EnlsvgPathPlanner object. It creates an internal grid representation
     * using the passed navigable_area with the passed resolution. It also pads the outer
     * boundaries of the grid as an obstacle using the boundary_margin_offset parameter.
     * It constructs the grid and sets the corresponding obstacles as "blocked".
     *
     * @param navigable_area         The total area that the path planner must path plan
     * inside
     * @param obstacles              A list of obstacles to consider when path planning
     * @param boundary_margin_offset Padding from the edges of the navigable_area to set
     * as an obstacle
     * @param resolution             The resolution of the internal representation of the
     * grid. Uses the same units as boundary_margin_offset
     */
    EnlsvgPathPlanner(const Rectangle &navigable_area,
                      const std::vector<ObstaclePtr> &obstacles,
                      double boundary_margin_offset = 0.0, double resolution = 0.09);

    /**
     * Returns a path that is an optimized path between start and end. Has no checking on
     * whether the start and end are valid and within field boundaries.
     *
     * @param start  start point
     * @param end    end point
     *
     * @return   Path that is the optimal path avoiding obstacles
     *           if no valid path, then return nullopt
     */
    std::optional<Path> findPath(const Point &start, const Point &end) const;

    /**
     * Returns internal resolution of the grid.
     *
     * @return double    size of grid cell (in the same units as passed into the
     * constructor)
     */
    inline double getResolution() const
    {
        return resolution;
    }

    /**
     * Compute the length of the path by summing the distance between consecutive points
     * start with the robot_position
     *
     * @param path_points The points on the path
     * @param robot_position The position of the robot
     *
     * @return the length of the path
     */
    static double pathLength(const std::vector<Point> &path_points,
                             const Point &robot_position);

    // 100 m upper limit for path length
    static constexpr double MAX_PATH_LENGTH = 100.0;

   private:
    using EnlsvgPath      = Pathfinding::Path;
    using EnlsvgGrid      = Pathfinding::Grid;
    using EnlsvgAlgorithm = Pathfinding::Enlsvg::Algorithm;
    using EnlsvgMemory    = Pathfinding::Enlsvg::Memory;

    /**
     * This struct is just the internal representation of a grid coordinate. This exists
     * to define equality operators and to minimize changes to extlibs.
     */
    struct EnlsvgPoint : Pathfinding::GridVertex
    {
        EnlsvgPoint() : Pathfinding::GridVertex() {}
        EnlsvgPoint(int x, int y) : Pathfinding::GridVertex(x, y) {}
        EnlsvgPoint(GridVertex p) : Pathfinding::GridVertex(p.x, p.y) {}

        inline bool operator==(const GridVertex &other) const
        {
            return (x == other.x) && (y == other.y);
        }

        inline bool operator!=(const GridVertex &other) const
        {
            return (x != other.x) || (y != other.y);
        }
    };

    /**
     * This struct defines a hash function to use for EnlsvgPoints.
     */
    struct HashEnlsvgPoint
    {
        size_t operator()(const EnlsvgPoint &p) const
        {
            std::size_t seed = 0;
            boost::hash_combine(seed, std::hash<int>{}(p.x));
            boost::hash_combine(seed, std::hash<int>{}(p.y));

            return seed;
        }
    };

    /**
     * Conversions from a Point <-> internal coordinate representation.
     *
     * @param p/gv               point to be converted
     *
     * @return EnlsvgPoint/Point converted point
     */
    EnlsvgPoint convertPointToEnlsvgPoint(const Point &p) const;
    Point convertEnlsvgPointToPoint(const EnlsvgPoint &gv) const;

    /**
     * Converts the internal representation of a path to a Path
     *
     * @param p          implenetation's representation of a path
     *
     * @return Point    converted Thunderbots Path
     */
    std::optional<Path> convertEnlsvgPathToPath(const EnlsvgPath &p) const;

    /**
     * Creates the obstacles in the grid from the given obstacles as well as
     * obstacle-izing regions from the edges of the grids based on the
     * grid_boundary_margin_offset.
     *
     * @param obstacles                      a list of obstacles
     * @param grid_boundary_margin_offset    an offset that represents the width of the
     * region from the edges of the grid to consider as an obstacle. Has the same units as
     * the resolution parameter in EnlsvgPathPlanner()
     */
    void createObstaclesInGrid(const std::vector<ObstaclePtr> &obstacles,
                               double grid_boundary_margin_offset) const;

    /**
     * Returns true if the internal representation of a coordinate is within the limits of
     * the grid.
     *
     * @param gv     a internal coordinate representation to check
     *
     * @return bool  true if the point is within the limits of the field, false otherwise
     */
    bool isCoordNavigable(const EnlsvgPoint &gv) const;

    /**
     * Returns the closest unblocked internal coordinate for a point. It may return the
     * given point itself.
     *
     * @param ep an internal point to investigate
     *
     * @return std::optional<EnlsvgPoint>    returns std::nullopt if the entire field is
     * blocked, or an internal grid coordinate representing a closest unblocked location
     */
    std::optional<EnlsvgPoint> findClosestUnblockedEnlsvgPoint(
        const EnlsvgPoint &ep) const;

    /*
     * Returns true if a given internal coordinate is blocked, false otherwise.
     *
     * @param ep an internal point to check
     *
     * @return   true if the point is blocked in the internal grid and false otherwise.
     * Also returns true if the given coordinate is out-of-bounds
     */
    bool isBlocked(const EnlsvgPoint &ep) const;

    // analogous to the size of a grid cell
    double resolution;

    // can be arbitrarily thought of as the width of the grid on the x-axis
    unsigned int num_grid_rows;
    // can be arbitrarily thought of as the height of the grid on the y-axis
    unsigned int num_grid_cols;

    // since the implementation depends on positive coordinates, this is the bottom-left
    // most coordinate
    Point origin;

    // max navigable coordinates
    int max_navigable_y_enlsvg_point;
    int max_navigable_x_enlsvg_point;

    // Required internal data structures
    std::unique_ptr<EnlsvgGrid> enlsvg_grid;
    std::unique_ptr<const EnlsvgAlgorithm> enlsvg_algo;
    std::unique_ptr<EnlsvgMemory> enlsvg_mem;
};
