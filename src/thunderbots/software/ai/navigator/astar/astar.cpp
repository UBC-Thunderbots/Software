#include "astar.h"

#include <exception>

#include "ai/intent/move_intent.h"
#include "ai/navigator/RobotObstacle.h"
#include "ai/primitive/move_primitive.h"

AStarNav::AStarNav(const Field& field):
    field_width(field.totalWidth()),
    field_length(field.totalLength())
{
    // TODO: allow configuring the field graph number of nodes with
    // dynamic parameters
    boost::array<size_t, 2> dims = {FIELD_NUM_LENGTH_NODES, FIELD_NUM_WIDTH_NODES};
    field_graph = std::make_unique<graph_t>(dims);
    x_step_size = field_length / FIELD_NUM_LENGTH_NODES;
    y_step_size = field_width / FIELD_NUM_WIDTH_NODES;
    min_x = - (field_length / 2);
    min_y = - (field_width / 2);
    graph_t::vertex_iterator v, vbegin, vend;
    for (boost::tie(v, vend) = vertices(*field_graph); v != vend; v++) {
        Point p = gridPointToPoint(*v);
        grid_points_list.emplace_back({p, *v});
    }
}

std::vector<std::unique_ptr<Primitive>> AStarNav::getAssignedPrimitives(
    const World &world, const std::vector<std::unique_ptr<Intent>> &assignedIntents) const
{
    return std::vector<std::unique_ptr<Primitive>>();
}

Point AStarNav::gridPointToPoint(const AStarNav::grid_point &grid_point) {
    Point p(min_x + grid_point[0] * x_step_size, min_y + grid_point[1] * y_step_size);
    return p;
}

AStarNav::grid_point AStarNav::nearestGridPoint(const Point &point) {
    // this is unfortunately O(n) w.r.t. number of vertices
    // try not to call this too much during the actual search
    auto closest_point_it = grid_points_list.begin();
    std::nth_element(grid_points_list.begin(), closest_point_it, grid_points_list.end(),
                    [point](const auto& a, const auto& b){
                        return dist(a.first, point) < dist(b.first, point);
                    });
    return closest_point_it->second;
}
