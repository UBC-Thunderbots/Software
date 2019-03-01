#pragma once

// Including these boost headers will have a performance hit on CLion
#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>

#include "ai/world/field.h"
#include "path_planner.h"
namespace AStar
{
    // edge cost type
    typedef double edge_cost_t;
    // 2D grid graph type
    typedef boost::grid_graph<2> GridGraph2D;
    // vertex type - a 2D point on a grid
    typedef GridGraph2D::vertex_descriptor GridVertex;

    // the factor to multiply violation distance by in the cost and heuristic functions
    // that A* uses
    // gigantic number empirically determined by running unit tests until obstacles were
    // actually avoided
    static constexpr double VIOLATION_SCALE_FACTOR = 1000.0f;

    class AStarGridGraph
    {
       public:
        AStarGridGraph() = delete;

        /**
         * Constructs a grid graph from a Field object.
         *
         * @param field the field to create a graph for
         * @param _grid_vertex_density how many vertices on the grid graph, per metre
         */
        explicit AStarGridGraph(const Field &field, const size_t _grid_vertex_density);

        /**
         * Converts a grid point to a Point on the field.
         *
         * @param grid_v the grid vertex to convert to a Point.
         */
        Point gridPointToPoint(const GridVertex &grid_v);

        /**
         * Finds the nearest vertex on the grid graph to the
         * given point.
         *
         * @param point the point to find a grid vertex close to it
         */
        GridVertex nearestGridVertex(const Point &point);

        /**
         * Returns a const reference to the grid graph itself.
         * @return the underlying grid graph.
         */
        const GridGraph2D &graph();

        /**
         * The distance between vertices on the grid graph.
         * @return the distance between vertices on the grid graph.
         */
        constexpr double gridVertexDistance();

       private:
        std::unique_ptr<GridGraph2D> field_graph;

        // this map is used to convert from the grid points that A* will
        // return, into Points that can be used for navigation
        std::vector<std::pair<Point, GridVertex>> grid_points_list;

        // the minimum and maximum x- and y-values on the field
        const double field_min_x, field_min_y;
        const size_t grid_vertex_density;
    };

    class AStarPathPlanner : public PathPlanner
    {
       public:
        /**
         * Constructs an AStarPathPlanner for a given field.
         * @param field the field
         * @param grid_vertex_density how many vertices on the grid graph per metre
         */
        explicit AStarPathPlanner(const Field &field, size_t grid_vertex_density);
        /**
         * Returns a path from start to dest if it is possible,
         * otherwise return std::nullopt
         *
         * @param violation_function a function that returns the violation distance for a
         * given point
         * @param start the start point
         * @param dest the destination point
         * @return a vector of Points that are a path from start to dest
         */
        std::optional<std::vector<Point>> findPath(
            const ViolationFunction &violation_function, const Point &start,
            const Point &dest) override;
        ~AStarPathPlanner() override = default;

       private:
        std::shared_ptr<AStarGridGraph> field_graph_ptr;
    };
}  // namespace AStar