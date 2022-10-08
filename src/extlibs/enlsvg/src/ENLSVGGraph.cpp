#include "extlibs/enlsvg/Pathfinding/ENLSVGGraph.h"

#include <algorithm>

#include "extlibs/enlsvg/Pathfinding/Grid.h"
#include "extlibs/enlsvg/Pathfinding/LineOfSightScanner.h"

namespace Pathfinding
{
    namespace ENLSVG
    {
        VisibilityGraph::VisibilityGraph(const Grid& grid,
                                         const LineOfSightScanner& scanner)
            : grid(grid),
              size_x(grid.size_x),
              size_y(grid.size_y),
              node_indexes_size_x(grid.size_x + 1),
              scanner(scanner)
        {
            // Initialise vertices (outer corners).
            node_indexes.resize(node_indexes_size_x * (size_y + 1));
            for (int y = 0; y <= size_y; ++y)
            {
                for (int x = 0; x <= size_x; ++x)
                {
                    if (grid.isOuterCorner(x, y))
                    {
                        node_indexes[y * node_indexes_size_x + x] = vertices.size();
                        vertices.push_back(GridVertex(x, y));
                    }
                }
            }
            vertices.shrink_to_fit();

            // Initialise SVG edges
            edge_lists.resize(vertices.size());
            ScannerStacks scanner_stacks;
            for (size_t i = 0; i < vertices.size(); ++i)
            {
                int cx = vertices[i].x;
                int cy = vertices[i].y;
                scanner.computeTautDirNeighbours(scanner_stacks, cx, cy);
                std::vector<GridVertex>& neighbours = scanner_stacks.neighbours;

                for (size_t j = 0; j < neighbours.size(); ++j)
                {
                    int nx = neighbours[j].x;
                    int ny = neighbours[j].y;

                    VertexID dest = node_indexes[ny * node_indexes_size_x + nx];
                    if (dest >= i)
                        continue;

                    connectEdge(i, dest, cx, cy, nx, ny);
                }
            }
            edges.shrink_to_fit();
            for (size_t i = 0; i < edge_lists.size(); ++i)
            {
                edge_lists[i].shrink_to_fit();
            }

            // Connect Taut Neighbours of Edges
            for (size_t i = 0; i < edges.size(); ++i)
            {
                EdgeData& edge                           = edges[i];
                std::vector<EdgeID>& taut_outgoing_edges = edge.taut_outgoing_edges;
                VertexID src                             = edge.source_vertex;
                VertexID dest                            = edge.dest_vertex;
                int s_x                                  = vertices[src].x;
                int s_y                                  = vertices[src].y;
                int e_x                                  = vertices[dest].x;
                int e_y                                  = vertices[dest].y;

                const std::vector<EdgeID>& outgoing_edges = edge_lists[dest];
                for (size_t j = 0; j < outgoing_edges.size(); ++j)
                {
                    EdgeID succ = outgoing_edges[j];
                    // assert edges[succ].source_vertex == dest
                    VertexID next = edges[succ].dest_vertex;
                    if (next == src)
                        continue;

                    int nx = vertices[next].x;
                    int ny = vertices[next].y;
                    if (grid.isTaut(s_x, s_y, e_x, e_y, nx, ny))
                    {
                        taut_outgoing_edges.push_back(succ);
                    }
                }
                taut_outgoing_edges.shrink_to_fit();
            }

            skip_edges.resize(vertices.size());
            buildHierarchy();
        }

        void VisibilityGraph::connectEdge(int i, int j, int x_i, int y_i, int x_j,
                                          int y_j)
        {
            EdgeID edge_ij = edges.size();
            edges.push_back(EdgeData(i, j, LEVEL_W));

            EdgeID edge_ji = edges.size();
            edges.push_back(EdgeData(j, i, LEVEL_W));

            // Note: The following must be true:
            // 1. edge_ij % 2 == 0
            // 2. edge_ji % 2 == 1
            // 3. edge_ij + 1 = edge_ji
            // We do this so that we can use the opposite(edgeId) function to get the
            // opposite edge.

            edge_lists[i].push_back(edge_ij);
            edge_lists[j].push_back(edge_ji);
        }

        void VisibilityGraph::buildHierarchy()
        {
            computeAllEdgeLevels();
            setupSkipEdges();
        }

        void VisibilityGraph::computeAllEdgeLevels()
        {
            std::vector<EdgeID> current_level_edges;
            std::vector<EdgeID> next_level_edges;

            std::vector<int> n_neighbours;
            n_neighbours.resize(edges.size());
            for (EdgeID i = 0; i < edges.size(); ++i)
            {
                int n           = edges[i].taut_outgoing_edges.size();
                n_neighbours[i] = n;
                if (n == 0)
                    current_level_edges.push_back(i);
            }

            int curr_level = 1;
            while (current_level_edges.size() > 0)
            {
                for (size_t i = 0; i < current_level_edges.size(); ++i)
                {
                    EdgeID curr = current_level_edges[i];
                    EdgeID opp  = opposite(curr);

                    edges[curr].level = curr_level;
                    edges[opp].level  = curr_level;

                    // Curr side must have no neighbours.
                    // Opp side may have neighbours.
                    const std::vector<EdgeID>& neighbours =
                        edges[opp].taut_outgoing_edges;
                    for (size_t j = 0; j < neighbours.size(); ++j)
                    {
                        EdgeID neighbour = opposite(neighbours[j]);
                        if (edges[neighbour].level != LEVEL_W)
                            continue;

                        --n_neighbours[neighbour];
                        if (n_neighbours[neighbour] == 0)
                        {
                            next_level_edges.push_back(neighbour);
                        }
                    }
                }
                current_level_edges.clear();
                std::swap(current_level_edges, next_level_edges);
                ++curr_level;
            }
        }

        void VisibilityGraph::setupSkipEdges()
        {
            std::vector<VertexID> skip_vertices;
            std::vector<bool> is_skip_vertex;
            is_skip_vertex.resize(vertices.size(), false);

            for (VertexID i = 0; i < vertices.size(); ++i)
            {
                const auto& edge_list = edge_lists[i];
                int n_level_w_edges   = 0;
                for (size_t j = 0; j < edge_list.size(); ++j)
                {
                    if (edges[edge_list[j]].level == LEVEL_W)
                    {
                        ++n_level_w_edges;
                        if (n_level_w_edges >= 3)
                        {
                            is_skip_vertex[i] = true;
                            skip_vertices.push_back(i);
                            break;
                        }
                    }
                }
            }

            for (size_t i = 0; i < skip_vertices.size(); ++i)
            {
                VertexID curr         = skip_vertices[i];
                const auto& edge_list = edge_lists[curr];

                for (size_t j = 0; j < edge_list.size(); ++j)
                {
                    if (edges[edge_list[j]].level != LEVEL_W)
                        continue;
                    // follow edge till next skip vertex.
                    double total_weight;
                    VertexID next_vertex;
                    VertexID immediate_next;
                    VertexID immediate_last;
                    followLevelWPathToNextSkipVertex(edge_list[j], total_weight,
                                                     next_vertex, immediate_next,
                                                     immediate_last, is_skip_vertex);
                    skip_edges[curr].push_back(SkipEdge(next_vertex, total_weight,
                                                        immediate_next, immediate_last));
                }
            }

            for (size_t i = 0; i < skip_edges.size(); ++i)
            {
                skip_edges[i].shrink_to_fit();
            }
        }

        void VisibilityGraph::followLevelWPathToNextSkipVertex(
            EdgeID first_edge, double& total_weight, VertexID& next_vertex,
            VertexID& immediate_next, VertexID& immediate_last,
            const std::vector<bool>& is_skip_vertex) const
        {
            EdgeID curr_edge = first_edge;
            total_weight     = weight(curr_edge);             // RETURN VALUE
            immediate_next   = edges[curr_edge].dest_vertex;  // RETURN VALUE

            while (!is_skip_vertex[edges[curr_edge].dest_vertex])
            {
                const auto& taut_outgoing_edges = edges[curr_edge].taut_outgoing_edges;

                // Find next outgoing level-W edge.
                for (size_t i = 0; i < taut_outgoing_edges.size(); ++i)
                {
                    EdgeID next_edge = taut_outgoing_edges[i];
                    if (edges[next_edge].level == LEVEL_W)
                    {
                        curr_edge = next_edge;
                        break;
                    }
                }
                total_weight += weight(curr_edge);
                // This should not infinite loop done correctly.
            }
            immediate_last = edges[curr_edge].source_vertex;  // RETURN VALUE
            next_vertex    = edges[curr_edge].dest_vertex;    // RETURN VALUE
        }

        void VisibilityGraph::markEdgesFrom(
            MarkedEdges& marked_edges, const int s_x, const int s_y,
            const std::vector<GridVertex>& neighbours) const
        {
            std::vector<EdgeID> edge_queue;

            for (size_t i = 0; i < neighbours.size(); ++i)
            {
                const GridVertex& u      = neighbours[i];
                const VertexID neighbour = node_indexes[u.y * node_indexes_size_x + u.x];
                const std::vector<EdgeID>& edge_list = edge_lists[neighbour];
                for (size_t j = 0; j < edge_list.size(); ++j)
                {
                    const EdgeID id      = edge_list[j];
                    const EdgeData& edge = edges[id];
                    // const GridVertex& u = vertices[edge.source_vertex];
                    const GridVertex& v = vertices[edge.dest_vertex];
                    if (grid.isTaut(s_x, s_y, u.x, u.y, v.x, v.y))
                    {
                        marked_edges.mark(id);
                        edge_queue.push_back(id);
                    }
                }
            }

            size_t head = 0;
            while (head < edge_queue.size())
            {
                EdgeID curr = edge_queue[head];

                const int curr_level            = edges[curr].level;
                const auto& taut_outgoing_edges = edges[curr].taut_outgoing_edges;
                bool skip_vertex                = isSkipVertex(edges[curr].dest_vertex);

                for (size_t j = 0; j < taut_outgoing_edges.size(); ++j)
                {
                    EdgeID next = taut_outgoing_edges[j];
                    if (marked_edges.is_marked[next])
                        continue;

                    // Should be (curr_level != LEVEL_W && edges[next].level > curr_level)
                    // || (!skip_vertex && edges[next].level == LEVEL_W) But we set
                    // LEVEL_W = INT_MAX, so, "curr_level != LEVEL_W" is no longer needed.
                    if ((edges[next].level > curr_level) ||
                        (!skip_vertex && edges[next].level == LEVEL_W))
                    {
                        marked_edges.mark(next);
                        edge_queue.push_back(next);
                    }
                }

                ++head;
            }
        }

        // Duplicate all markings. Call this after calling markEdgesFrom from both start
        // and goal.
        void VisibilityGraph::markBothWays(MarkedEdges& marked_edges) const
        {
            std::vector<EdgeID>& marked_indexes = marked_edges.marked_indexes;
            size_t n_marked                     = marked_indexes.size();  // freeze size.
            for (size_t i = 0; i < n_marked; ++i)
            {
                EdgeID opp = opposite(marked_indexes[i]);
                if (!marked_edges.is_marked[opp])
                    marked_edges.mark(opp);
            }
        }


        void VisibilityGraph::printStatistics() const
        {
            int n_vertices = vertices.size();
            int n_edges    = edges.size();

            int total_edge_degree = 0;
            for (auto& edge : edges)
            {
                total_edge_degree += edge.taut_outgoing_edges.size();
            }
            float average_edge_degree = (float)total_edge_degree / n_edges;

            int total_vertex_degree = 0;
            for (auto& edge_list : edge_lists)
            {
                total_vertex_degree += edge_list.size();
            }
            float average_vertex_degree = (float)total_vertex_degree / n_vertices;

            int n_skip_edges = 0;
            for (auto& skip_edge : skip_edges)
            {
                n_skip_edges += skip_edge.size();
            }

            std::cout << "n_vertices: " << n_vertices << std::endl;
            std::cout << "n_edges: " << n_edges << std::endl;
            std::cout << "n_skip_edges: " << n_skip_edges << std::endl;
            std::cout << "total_edge_degree: " << total_edge_degree << std::endl;
            std::cout << "total_vertex_degree: " << total_vertex_degree << std::endl;
            std::cout << "average_edge_degree: " << average_edge_degree << std::endl;
            std::cout << "average_vertex_degree: " << average_vertex_degree << std::endl;
        }
    }  // namespace ENLSVG
}  // namespace Pathfinding
