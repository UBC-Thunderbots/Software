#include "astar.h"

#include <boost/property_map/function_property_map.hpp>
#include <boost/unordered_map.hpp>

#include "geom/util.h"

namespace AStar
{
    class AStarHeuristic : public boost::astar_heuristic<GridGraph2D, edge_cost_t>
    {
       public:
        AStarHeuristic() = delete;

        /**
         * Constructs an AStarHeuristic for a given graph and destination point.
         * @param _graph the graph
         * @param _violation_function a function that returns the violation distance for a
         * given point
         * @param _dest the destination point
         */
        explicit AStarHeuristic(const std::shared_ptr<AStarGridGraph> &_graph,
                                const ViolationFunction &_violation_function,
                                const Point &_dest);
        edge_cost_t operator()(GridVertex gp);

       private:
        std::shared_ptr<AStarGridGraph> graph;
        const Point dest_point;
        const ViolationFunction violation_function;
    };

    struct FoundGoalException
    {
    };

    class AStarVertexVisitor : public boost::default_astar_visitor
    {
       public:
        AStarVertexVisitor() = delete;

        /**
         * Construct a visitor for the given destination graph vertex.
         * @param _dest destinaton graph vertex
         */
        explicit AStarVertexVisitor(GridVertex _dest);

        /**
         * Throws FoundGoal if the destination vertex is reached.
         * @param grid_v the grid vertex that may or may not be the destination
         * @param graph the graph
         */
        void examine_vertex(GridVertex grid_v, const GridGraph2D &graph);

       private:
        GridVertex dest;
    };

    // a hash function for a grid vertex, necessary for all the maps
    // that boost::astar_search uses
    struct grid_vertex_hash : std::unary_function<GridVertex, std::size_t>
    {
        std::size_t operator()(const GridVertex &gp) const;
    };
}  // namespace AStar

std::size_t AStar::grid_vertex_hash::operator()(const GridVertex &gp) const
{
    std::size_t seed = 0;
    boost::hash_combine(seed, gp[0]);
    boost::hash_combine(seed, gp[1]);
}

AStar::AStarGridGraph::AStarGridGraph(const Field &field,
                                      const size_t _grid_vertex_density)
    : field_min_x(-(field.length() / 2)),
      field_min_y(-(field.width() / 2)),
      grid_vertex_density(_grid_vertex_density)
{
    size_t field_length_nodes                = field.totalLength() * grid_vertex_density;
    size_t field_width_nodes                 = field.totalWidth() * grid_vertex_density;
    boost::array<size_t, 2> graph_dimensions = {field_length_nodes, field_width_nodes};

    field_graph = std::make_unique<GridGraph2D>(graph_dimensions);

    GridGraph2D::vertex_iterator v, vbegin, vend;
    for (boost::tie(v, vend) = vertices(*field_graph); v != vend; v++)
    {
        Point p = gridPointToPoint(*v);
        grid_points_list.emplace_back(p, *v);
    }
}

Point AStar::AStarGridGraph::gridPointToPoint(const GridVertex &grid_v)
{
    double step_size = 1.0f / grid_vertex_density;
    Point p(field_min_x + grid_v[0] * step_size, field_min_y + grid_v[1] * step_size);
    return p;
}

AStar::GridVertex AStar::AStarGridGraph::nearestGridVertex(const Point &point)
{
    // this is unfortunately O(n) w.r.t. number of vertices
    // try not to call this too much during the actual search
    auto closest_point_it = grid_points_list.begin();
    std::nth_element(grid_points_list.begin(), closest_point_it, grid_points_list.end(),
                     [point](const auto &a, const auto &b) {
                         return dist(a.first, point) < dist(b.first, point);
                     });
    return closest_point_it->second;
}

const AStar::GridGraph2D &AStar::AStarGridGraph::graph()
{
    return *field_graph;
}

constexpr double AStar::AStarGridGraph::gridVertexDistance()
{
    return 1.0f / grid_vertex_density;
}

AStar::AStarHeuristic::AStarHeuristic(
    const std::shared_ptr<AStar::AStarGridGraph> &_graph,
    const ViolationFunction &_violation_function, const Point &_dest)
    : graph(_graph), violation_function(_violation_function), dest_point(_dest)
{
}

AStar::edge_cost_t AStar::AStarHeuristic::operator()(AStar::GridVertex gp)
{
    // TODO: scale violation component based on grid density
    Point p = graph->gridPointToPoint(gp);
    double p_violation =
        VIOLATION_SCALE_FACTOR * graph->gridVertexDistance() * violation_function(p);
    return dist(dest_point, p) + p_violation;
}

AStar::AStarVertexVisitor::AStarVertexVisitor(AStar::GridVertex _dest) : dest(_dest) {}

void AStar::AStarVertexVisitor::examine_vertex(GridVertex grid_v,
                                               const GridGraph2D &graph)
{
    if (grid_v == dest)
    {
        // throw an exception to indicate that the goal node was reached
        throw AStar::FoundGoalException();
    }
}

std::optional<std::vector<Point>> AStar::AStarPathPlanner::findPath(
    const ViolationFunction &violation_function, const Point &start, const Point &dest)
{
    // create a map that dynamically generates edge weights as we traverse the grid graph
    auto edge_weights = boost::make_function_property_map<GridGraph2D::edge_descriptor>(
        [this, violation_function](GridGraph2D::edge_descriptor edge) -> edge_cost_t {
            // TODO: scale violation based on grid density
            // TODO: memoizing this may improve performance eventually
            // cost of an edge is the distance between grid points
            double cost            = this->field_graph_ptr->gridVertexDistance();
            Point edge_start_point = this->field_graph_ptr->gridPointToPoint(edge.first);
            Point edge_dest_point  = this->field_graph_ptr->gridPointToPoint(edge.second);
            // increase the cost by the increase in violation between the start and end
            // points of the edge
            double edge_start_violation = VIOLATION_SCALE_FACTOR *
                                          field_graph_ptr->gridVertexDistance() *
                                          violation_function(edge_start_point);

            double edge_dest_violation = VIOLATION_SCALE_FACTOR *
                                         field_graph_ptr->gridVertexDistance() *
                                         violation_function(edge_start_point);

            cost = edge_dest_violation - edge_start_violation;
            return cost;
        });

    GridVertex start_v = field_graph_ptr->nearestGridVertex(start);
    GridVertex dest_v  = field_graph_ptr->nearestGridVertex(dest);
    AStarHeuristic heuristic(field_graph_ptr, violation_function, dest);

    using pred_map     = boost::unordered_map<GridVertex, GridVertex, grid_vertex_hash>;
    using distance_map = boost::unordered_map<GridVertex, edge_cost_t>;

    pred_map preds;
    distance_map dists;
    boost::associative_property_map<pred_map> pred_pmap(preds);
    boost::associative_property_map<distance_map> dist_pmap(dists);

    AStarVertexVisitor visitor(dest_v);

    std::list<GridVertex> soln_grid_points;
    bool soln_found = false;

    try
    {
        boost::astar_search(field_graph_ptr->graph(), start_v, heuristic,
                            boost::weight_map(edge_weights)
                                .predecessor_map(pred_pmap)
                                .distance_map(dist_pmap)
                                .visitor(visitor));
    }
    catch (FoundGoalException fg)
    {
        soln_found = true;
        for (GridVertex gp = dest_v; gp != start_v; gp = preds[gp])
        {
            soln_grid_points.push_front(gp);
        }
    }

    if (!soln_found)
    {
        return std::nullopt;
    }

    std::vector<Point> path(soln_grid_points.size() + 2);
    path[0]               = start;
    path[path.size() - 1] = dest;

    // fill the path from the 2nd item to the 2nd-last item with points that A* found
    std::transform(
        soln_grid_points.begin(), soln_grid_points.end(), path.begin() + 1,
        [this](const auto &gp) { return field_graph_ptr->gridPointToPoint(gp); });

    return std::make_optional(path);
}

AStar::AStarPathPlanner::AStarPathPlanner(const Field &field,
                                          const size_t grid_vertex_density)
    : field_graph_ptr(new AStarGridGraph(field, grid_vertex_density))
{
    if (grid_vertex_density < 1)
    {
        throw std::invalid_argument(
            "grid_vertex_density == 0! "
            "The graph must have vertices!");
    }
}
