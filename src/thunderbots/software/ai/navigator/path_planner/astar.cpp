//
// Created by jordan on 2/26/19.
//
#include <boost/unordered_map.hpp>
#include <boost/property_map/function_property_map.hpp>
#include "astar.h"
#include "geom/util.h"

// struct grid_point_hash
std::size_t AStar::grid_point_hash::operator() (const grid_point& gp) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, gp[0]);
    boost::hash_combine(seed, gp[1]);
}

// AStarGridGraph
AStar::AStarGridGraph::AStarGridGraph(const Field &field) :
field_min_x(- (field.length() / 2)),
field_min_y(- (field.width() / 2))
{
    // TODO: allow configuring the field graph number of nodes with
    // dynamic parameters
    size_t field_length_nodes = field.totalLength() * GRID_POINT_DENSITY;
    size_t field_width_nodes = field.totalWidth() * GRID_POINT_DENSITY;
    boost::array<size_t, 2> graph_dimensions = {field_length_nodes, field_width_nodes};

    graph_t::vertex_iterator v, vbegin, vend;
    for (boost::tie(v, vend) = vertices(*field_graph); v != vend; v++)
    {
        Point p = gridPointToPoint(*v);
        grid_points_list.emplace_back(p, *v);
    }
}

Point AStar::AStarGridGraph::gridPointToPoint(const grid_point &grid_point)
{
    double step_size = 1.0f / GRID_POINT_DENSITY;
    Point p(field_min_x + grid_point[0] * step_size, field_min_y + grid_point[1] * step_size);
    return p;
}

AStar::grid_point AStar::AStarGridGraph::nearestGridPoint(const Point &point)
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

const AStar::graph_t& AStar::AStarGridGraph::graph() {
    return *field_graph;
}

constexpr double AStar::AStarGridGraph::gridPointDistance() {
    return 1.0f / GRID_POINT_DENSITY;
}

//AStarHeuristic
AStar::AStarHeuristic::AStarHeuristic(
        const std::shared_ptr<AStar::AStarGridGraph> &_graph, const Point& _dest)
        : graph(_graph), dest_point(_dest)
{
}

AStar::cost_t AStar::AStarHeuristic::operator()(AStar::grid_point gp)
{
    // TODO: add properly scaled obstacle component to heuristic
    Point p = graph->gridPointToPoint(gp);
    return dist(dest_point, p);
}

// AStarVertexVisitor
AStar::AStarVertexVisitor::AStarVertexVisitor(AStar::grid_point _dest):
        dest(_dest)
{}

void AStar::AStarVertexVisitor::examine_vertex(grid_point gp, const graph_t& graph)
{
    if (gp == dest)
    {
        // throw an exception because the people who came up with boost
        // are crazy
        throw AStar::FoundGoal();
    }
}

std::optional<std::vector<Point>>
AStar::AStarPathPlanner::findPath(const World &world, const Point &start, const Point &dest) {

    // create a map that dynamically generates edge weights as we traverse the grid graph
    auto edge_weights = boost::make_function_property_map<graph_t::edge_descriptor>(
            [this](graph_t::edge_descriptor edge) -> cost_t {
                // TODO: find edge costs including obstacles and whatnot
                // TODO: memoizing this may improve performance eventually
                // cost of an edge is the distance between grid points
                return this->field_graph_ptr->gridPointDistance();
            }
            );

    grid_point start_v = field_graph_ptr->nearestGridPoint(start);
    grid_point dest_v = field_graph_ptr->nearestGridPoint(dest);
    AStarHeuristic heuristic(field_graph_ptr, dest);

    using pred_map = boost::unordered_map<grid_point, grid_point, grid_point_hash>;
    using distance_map = boost::unordered_map<grid_point, cost_t>;

    pred_map preds;
    distance_map dists;
    boost::associative_property_map<pred_map> pred_pmap(preds);
    boost::associative_property_map<distance_map> dist_pmap(dists);

    AStarVertexVisitor visitor(dest_v);

    std::list<grid_point> soln_grid_points;

    try {
        boost::astar_search(field_graph_ptr->graph(), start_v, heuristic,
                            boost::weight_map(edge_weights).
                            predecessor_map(pred_pmap).
                            distance_map(dist_pmap).
                            visitor(visitor));
    } catch (FoundGoal fg) {
        for (grid_point gp = dest_v; gp != start_v; gp = preds[gp]) {
            soln_grid_points.push_front(gp);
        }
    }

    if (soln_grid_points.empty()) {
        return std::make_optional<std::vector<Point>>();
    }

    std::vector<Point> path(soln_grid_points.size() + 2);
    path[0] = start;
    path[soln_grid_points.size() - 1] = dest;

    std::transform(soln_grid_points.begin(), soln_grid_points.end(), path.begin() + 1,
                    [this](const auto& gp){
                        return field_graph_ptr->gridPointToPoint(gp);
                    });

    return std::make_optional(path);
}
