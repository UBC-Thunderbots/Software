//
// Created by jordan on 2/26/19.
//
#include "astar.h"

#include <boost/property_map/function_property_map.hpp>
#include <boost/unordered_map.hpp>

#include "geom/util.h"

// struct grid_vertex_hash
std::size_t AStar::grid_vertex_hash::operator()(const GridVertex &gp) const
{
    std::size_t seed = 0;
    boost::hash_combine(seed, gp[0]);
    boost::hash_combine(seed, gp[1]);
}

// AStarGridGraph
AStar::AStarGridGraph::AStarGridGraph(const Field &field,
                                      const size_t _grid_vertex_density)
    : field_min_x(-(field.length() / 2)),
      field_min_y(-(field.width() / 2)),
      grid_vertex_density(_grid_vertex_density)
{
    // TODO: allow configuring the field graph number of nodes with
    // dynamic parameters
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

// AStarHeuristic
AStar::AStarHeuristic::AStarHeuristic(
    const std::shared_ptr<AStar::AStarGridGraph> &_graph, const Point &_dest)
    : graph(_graph), dest_point(_dest)
{
}

AStar::edge_cost_t AStar::AStarHeuristic::operator()(AStar::GridVertex gp)
{
    // TODO: add properly scaled obstacle component to heuristic
    Point p = graph->gridPointToPoint(gp);
    return dist(dest_point, p);
}

// AStarVertexVisitor
AStar::AStarVertexVisitor::AStarVertexVisitor(AStar::GridVertex _dest) : dest(_dest) {}

void AStar::AStarVertexVisitor::examine_vertex(GridVertex grid_v,
                                               const GridGraph2D &graph)
{
    if (grid_v == dest)
    {
        // throw an exception because the people who came up with boost
        // are crazy
        throw AStar::FoundGoal();
    }
}

std::optional<std::vector<Point>> AStar::AStarPathPlanner::findPath(const World &world,
                                                                    const Point &start,
                                                                    const Point &dest)
{
    // create a map that dynamically generates edge weights as we traverse the grid graph
    auto edge_weights = boost::make_function_property_map<GridGraph2D::edge_descriptor>(
        [this](GridGraph2D::edge_descriptor edge) -> edge_cost_t {
            // TODO: find edge costs including obstacles and whatnot
            // TODO: memoizing this may improve performance eventually
            // cost of an edge is the distance between grid points
            return this->field_graph_ptr->gridVertexDistance();
        });

    GridVertex start_v = field_graph_ptr->nearestGridVertex(start);
    GridVertex dest_v  = field_graph_ptr->nearestGridVertex(dest);
    AStarHeuristic heuristic(field_graph_ptr, dest);

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
    catch (FoundGoal fg)
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
