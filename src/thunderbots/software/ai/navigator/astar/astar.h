#pragma once

// sincere apologies for quadrupling compile times
#include <boost/graph/grid_graph.hpp>

#include "ai/navigator/navigator.h"

// TODO: move the actual A* implementation into its own class so that it can
// be tested

class AStarNav : public Navigator {
public:
    /**
     * Creates a new Navigator that uses A*
     * to generate paths
     */
    AStarNav() = delete;
    // we need field dimensions to have consistent sized nodes
    explicit AStarNav(const Field& field);

    std::vector<std::unique_ptr<Primitive>> getAssignedPrimitives(
            const World &world,
            const std::vector<std::unique_ptr<Intent>> &assignedIntents) const override;

private:
    // TODO: these should be dynamic parameters
    static constexpr unsigned long FIELD_NUM_LENGTH_NODES = 960;
    static constexpr unsigned long FIELD_NUM_WIDTH_NODES = 660;
    typedef double cost_t;
    typedef boost::grid_graph<2> graph_t;
    typedef graph_t::vertex_descriptor grid_point;

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

    // TODO: create functions to go from grid point to field point, and vice versa
    std::vector<Point> findPath(const Point& start, const Point& dest);
    Point gridPointToPoint(const grid_point &grid_point);
    grid_point nearestGridPoint(const Point& point);
};
