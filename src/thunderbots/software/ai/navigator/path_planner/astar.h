#pragma once

// I apologize if your CLion slows to a crawl
#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>
#include "ai/world/field.h"
#include "path_planner.h"
namespace AStar {
    // TODO: move a lot of stuff out of this header

    // number of graph vertices per metre
    // TODO: make this a dynamic parameter
    static constexpr unsigned long GRID_POINT_DENSITY = 20;
    // edge cost type
    typedef double cost_t;
    // 2D grid graph type
    typedef boost::grid_graph<2> graph_t;
    // vertex type - a 2D point on a grid
    typedef graph_t::vertex_descriptor grid_point;

    struct grid_point_hash : std::unary_function<grid_point, std::size_t> {
        std::size_t operator() (const grid_point& gp) const;
    };

    class AStarGridGraph
    {
    public:
        AStarGridGraph() = delete;

        explicit AStarGridGraph(const Field &field);

        Point gridPointToPoint(const grid_point &grid_point);

        grid_point nearestGridPoint(const Point &point);

        const graph_t& graph();

        constexpr double gridPointDistance();

    private:
        std::unique_ptr<graph_t> field_graph;
        // this map is used to convert from the grid points that A* will
        // return, into Points that can be used for navigation
        std::vector<std::pair<Point, grid_point>> grid_points_list;
        const double field_min_x, field_min_y;
    };

    class AStarHeuristic : public boost::astar_heuristic<graph_t, cost_t>
    {
    public:
        AStarHeuristic() = delete;
        explicit AStarHeuristic(const std::shared_ptr<AStarGridGraph> &_graph,
                                const Point& _dest);
        cost_t operator()(grid_point gp);

    private:
        std::shared_ptr<AStarGridGraph> graph;
        const Point dest_point;
    };

    struct FoundGoal {};

    class AStarVertexVisitor : public boost::default_astar_visitor
    {
    public:
        AStarVertexVisitor() = delete;
        explicit AStarVertexVisitor(grid_point _dest);
        void examine_vertex(grid_point gp, const graph_t& graph);
    private:
        grid_point dest;
    };

    class AStarPathPlanner : public PathPlanner {
    public:
        std::optional<std::vector<Point>>
        findPath(const World &world, const Point &start, const Point &dest) override;
        ~AStarPathPlanner() override = default;
    private:
        std::shared_ptr<AStarGridGraph> field_graph_ptr;
    };
}