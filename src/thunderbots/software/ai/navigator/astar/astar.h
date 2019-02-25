#pragma once

// sincere apologies for quadrupling compile times
#include <boost/graph/astar_search.hpp>
#include <boost/graph/grid_graph.hpp>

#include "ai/navigator/navigator.h"

// TODO: move the actual A* implementation into its own class so that it can
// be tested
namespace AStar
{
    static constexpr unsigned long FIELD_NUM_LENGTH_NODES = 960;
    static constexpr unsigned long FIELD_NUM_WIDTH_NODES  = 660;
    typedef double cost_t;
    typedef boost::grid_graph<2> graph_t;
    typedef graph_t::vertex_descriptor grid_point;

    class AStarGridGraph
    {
       public:
        AStarGridGraph() = delete;

        explicit AStarGridGraph(const Field &field);

        Point gridPointToPoint(const grid_point &grid_point);

        grid_point nearestGridPoint(const Point &point);

       private:
        std::unique_ptr<graph_t> field_graph;
        // this map is used to convert from the grid points that A* will
        // return, into Points that can be used for navigation
        std::vector<std::pair<Point, grid_point>> grid_points_list;

        double field_width, field_length;
        double min_x, min_y;
        // node length/width
        // this is a member field because we don't want to recalculate it
        // every time gridPointToPoint is called
        double x_step_size, y_step_size;
    };

    class AStarHeuristic : public boost::astar_heuristic<grid_point, cost_t>
    {
    public:
        AStarHeuristic() = delete;
        explicit AStarHeuristic(const std::shared_ptr<AStarGridGraph> _graph, grid_point _dest);
        cost_t operator() (grid_point gp);
    private:
        std::shared_ptr<AStarGridGraph> graph;
        const grid_point dest;
        const Point dest_point;
    };

    class AStarNav : public Navigator
    {
       public:
        /**
         * Creates a new Navigator that uses A*
         * to generate paths
         */
        AStarNav() = delete;

        // we need field dimensions to have consistent sized nodes
        explicit AStarNav(const Field &field);

        std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
            const World &world,
            const std::vector<std::unique_ptr<Intent>> &assignedIntents) const override;

       private:
        // TODO: create functions to go from grid point to field point, and vice versa
        std::vector<Point> findPath(const Point &start, const Point &dest);

        // this is only a shared pointer so that AStarHeuristic can have a view
        // of the graph through a weak_ptr
        std::shared_ptr<AStarGridGraph> field_graph_ptr;

    };
}  // namespace AStar