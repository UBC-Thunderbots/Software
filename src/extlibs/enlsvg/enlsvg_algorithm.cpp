#include "extlibs/enlsvg/enlsvg_algorithm.h"

#include <algorithm>

#include "extlibs/enlsvg/enlsvg_graph.h"
#include "extlibs/enlsvg/grid.h"
#include "extlibs/enlsvg/indirect_heap.h"
#include "extlibs/enlsvg/line_of_sight_scanner.h"

namespace Pathfinding
{
    namespace Enlsvg
    {
        // negatePar makes parent pointers negative to indicate that the parent is a
        // skip-edge. note that negatePar(negatePar(k)) == k.
        inline VertexID negatePar(VertexID k)
        {
            return -k - 1;
        }
        // restorePar returns the parent pointer to its original value.
        inline VertexID restorePar(VertexID k)
        {
            return k < 0 ? -k - 1 : k;
        }

        Memory::Memory(const Algorithm& algo)
            : n_edges(algo.nEdges()),
              n_nodes(algo.nVertices()),
              marked_edges(n_edges),
              pq(n_nodes)
        {
            const size_t nNodes = algo.nVertices();
            nodes.resize(nNodes);
            ticket_check.resize(nNodes, 0);
            ticket_number = 1;
        }


        Algorithm::Algorithm(const Grid& grid)
            : grid(grid), scanner(grid), graph(grid, scanner)
        {
        }


        Path Algorithm::computeSVGPath(Memory& memory, const int s_x, const int s_y,
                                       const int e_x, const int e_y,
                                       ParentPtrs* parent_ptrs) const
        {
            if (!memory.validate(graph))
                std::cout << "VALIDATION ERROR: MEMORY USED IS INVALID" << std::endl;

            // START: SPECIAL CASES - Handle special cases first.
            if (s_x == e_x && s_y == e_y)
            {
                Path path;
                path.push_back(GridVertex(s_x, s_y));
                return path;
            }
            else if (grid.lineOfSight(s_x, s_y, e_x, e_y))
            {
                Path path;
                path.push_back(GridVertex(s_x, s_y));
                path.push_back(GridVertex(e_x, e_y));
                return path;
            }
            // END: SPECIAL CASES


            const size_t nNodes = graph.vertices.size();
            memory.initialise();
            IndirectHeap& pq = memory.pq;
            pq.reinitialise();

            double goal_distance = POS_INF;
            VertexID goal_parent = NO_PARENT;
            {  // START: INITIALISATION
                ScannerStacks& data = memory.scanner_stacks;
                {
                    const VertexID start_index = graph.nodeIndex(s_x, s_y);
                    // Add all visible neighbours of start vertex
                    scanner.computeAllDirNeighbours(data, s_x, s_y);
                    const std::vector<GridVertex>& neighbours = data.neighbours;
                    for (size_t i = 0; i < neighbours.size(); ++i)
                    {
                        const GridVertex& vn = neighbours[i];
                        VertexID neighbour   = graph.nodeIndex(vn.x, vn.y);

                        double dist = grid.euclideanDistance(s_x, s_y, vn.x, vn.y);
                        memory.setDistance(neighbour, dist);
                        pq.decreaseKey(neighbour, dist + heuristic(neighbour, e_x, e_y));
                    }
                    if (start_index != -1)
                    {
                        // If start vertex is a VG node, mark it as visited so we don't
                        // waste time on it.
                        memory.setVisited(start_index, true);
                    }
                }
                {
                    const VertexID goal_index = graph.nodeIndex(e_x, e_y);
                    // Add all visible neighbours of goal vertex
                    scanner.computeAllDirNeighbours(data, e_x, e_y);
                    const std::vector<GridVertex>& neighbours = data.neighbours;
                    for (size_t i = 0; i < neighbours.size(); ++i)
                    {
                        const GridVertex& vn = neighbours[i];
                        VertexID neighbour   = graph.nodeIndex(vn.x, vn.y);
                        memory.setEdgeWeightToGoal(
                            neighbour, grid.euclideanDistance(vn.x, vn.y, e_x, e_y));
                    }
                    if (goal_index != -1)
                    {
                        // If goal vertex is a VG node, mark it as visited so we don't
                        // waste time on it.
                        memory.setVisited(goal_index, true);
                    }
                }
            }  // END: INITIALISATION

            while (pq.size() > 0)
            {
                if (goal_distance <= pq.getMinValue())
                {
                    break;  // Reached Goal.
                }
                VertexID curr              = pq.popMinIndex();
                const double curr_distance = memory.distance(curr);
                const VertexID curr_parent = restorePar(memory.parent(curr));
                memory.setVisited(curr, true);

                const std::vector<EdgeID>& neighbours = graph.edge_lists[curr];
                for (size_t i = 0; i < neighbours.size(); ++i)
                {
                    const auto& edge = graph.edges[neighbours[i]];
                    VertexID dest    = edge.dest_vertex;
                    if (memory.visited(dest))
                        continue;
                    double weight = graph.weight(edge);

                    double dest_distance = curr_distance + weight;
                    if (dest_distance < memory.distance(dest) &&
                        isTaut(curr_parent, curr, dest))
                    {
                        memory.setParent(dest, curr);
                        memory.setDistance(dest, dest_distance);
                        pq.decreaseKey(dest, dest_distance + heuristic(dest, e_x, e_y));
                    }
                }

                if (memory.edgeWeightToGoal(curr) != -1.0)
                {
                    double dest_distance = curr_distance + memory.edgeWeightToGoal(curr);
                    if (dest_distance < goal_distance)
                    {
                        goal_distance = dest_distance;  // heuristic of goal should be 0
                        goal_parent   = curr;
                    }
                }
            }

            if (parent_ptrs != nullptr)
                setParentPointers(memory, goal_parent, s_x, s_y, e_x, e_y, parent_ptrs);
            return getPath(memory, goal_parent, s_x, s_y, e_x, e_y);
        }



        Path Algorithm::computePath(Memory& memory, const int s_x, const int s_y,
                                    const int e_x, const int e_y,
                                    ParentPtrs* parent_ptrs) const
        {
            if (!memory.validate(graph))
                std::cout << "VALIDATION ERROR: MEMORY USED IS INVALID" << std::endl;

            // START: SPECIAL CASES - Handle special cases first.
            if (s_x == e_x && s_y == e_y)
            {
                Path path;
                path.push_back(GridVertex(s_x, s_y));
                return path;
            }
            else if (grid.lineOfSight(s_x, s_y, e_x, e_y))
            {
                Path path;
                path.push_back(GridVertex(s_x, s_y));
                path.push_back(GridVertex(e_x, e_y));
                return path;
            }
            // END: SPECIAL CASES

            const size_t nNodes = graph.vertices.size();
            memory.initialise();
            IndirectHeap& pq = memory.pq;
            pq.reinitialise();

            MarkedEdges& marked_edges = memory.marked_edges;
            double goal_distance      = POS_INF;
            VertexID goal_parent      = NO_PARENT;
            {  // START: INITIALISATION
                ScannerStacks& data = memory.scanner_stacks;
                {
                    const VertexID start_index = graph.nodeIndex(s_x, s_y);
                    // Add all visible neighbours of start vertex
                    scanner.computeAllDirNeighbours(data, s_x, s_y);
                    const std::vector<GridVertex>& neighbours = data.neighbours;
                    for (size_t i = 0; i < neighbours.size(); ++i)
                    {
                        const GridVertex& vn = neighbours[i];
                        VertexID neighbour   = graph.nodeIndex(vn.x, vn.y);

                        double dist = grid.euclideanDistance(s_x, s_y, vn.x, vn.y);
                        memory.setDistance(neighbour, dist);
                        pq.decreaseKey(neighbour, dist + heuristic(neighbour, e_x, e_y));
                    }
                    if (start_index != -1)
                    {
                        // If start vertex is a VG node, mark it as visited so we don't
                        // waste time on it.
                        memory.setVisited(start_index, true);
                    }
                    graph.markEdgesFrom(marked_edges, s_x, s_y, neighbours);
                }
                {
                    const VertexID goal_index = graph.nodeIndex(e_x, e_y);
                    // Add all visible neighbours of goal vertex
                    scanner.computeAllDirNeighbours(data, e_x, e_y);
                    const std::vector<GridVertex>& neighbours = data.neighbours;
                    for (size_t i = 0; i < neighbours.size(); ++i)
                    {
                        const GridVertex& vn = neighbours[i];
                        VertexID neighbour   = graph.nodeIndex(vn.x, vn.y);
                        memory.setEdgeWeightToGoal(
                            neighbour, grid.euclideanDistance(vn.x, vn.y, e_x, e_y));
                    }
                    if (goal_index != -1)
                    {
                        // If goal vertex is a VG node, mark it as visited so we don't
                        // waste time on it.
                        memory.setVisited(goal_index, true);
                    }
                    graph.markEdgesFrom(marked_edges, e_x, e_y, neighbours);
                }
            }  // END: INITIALISATION
            graph.markBothWays(marked_edges);

            while (pq.size() > 0)
            {
                if (goal_distance <= pq.getMinValue())
                {
                    break;  // Reached Goal, or min value = POS_INF (can't find goal)
                }
                VertexID curr              = pq.popMinIndex();
                const double curr_distance = memory.distance(curr);
                const VertexID curr_parent = restorePar(memory.parent(curr));
                memory.setVisited(curr, true);

                // Traverse marked edges
                const std::vector<EdgeID>& neighbours = graph.edge_lists[curr];
                for (size_t i = 0; i < neighbours.size(); ++i)
                {
                    const EdgeID edge_id = neighbours[i];
                    if (!marked_edges.is_marked[edge_id])
                        continue;
                    const auto& edge = graph.edges[edge_id];
                    VertexID dest    = edge.dest_vertex;
                    if (memory.visited(dest))
                        continue;
                    double weight = graph.weight(edge);

                    double dest_distance = curr_distance + weight;
                    if (dest_distance < memory.distance(dest) &&
                        isTaut(curr_parent, curr, dest))
                    {
                        memory.setParent(dest, curr);
                        memory.setDistance(dest, dest_distance);
                        pq.decreaseKey(dest, dest_distance + heuristic(dest, e_x, e_y));
                    }
                }

                // Traverse skip edges
                const std::vector<SkipEdge>& skip_edges = graph.skip_edges[curr];
                for (size_t i = 0; i < skip_edges.size(); ++i)
                {
                    const SkipEdge& edge = skip_edges[i];
                    VertexID dest        = edge.next;
                    if (memory.visited(dest))
                        continue;
                    double weight = edge.weight;

                    double dest_distance = curr_distance + weight;
                    if (dest_distance < memory.distance(dest) &&
                        isTaut(curr_parent, curr, edge.immediate_next))
                    {
                        memory.setParent(dest, negatePar(edge.immediate_last));
                        memory.setDistance(dest, dest_distance);
                        pq.decreaseKey(dest, dest_distance + heuristic(dest, e_x, e_y));
                    }
                }

                if (memory.edgeWeightToGoal(curr) != -1.0)
                {
                    double dest_distance = curr_distance + memory.edgeWeightToGoal(curr);
                    if (dest_distance < goal_distance)
                    {
                        goal_distance = dest_distance;  // heuristic of goal should be 0
                        goal_parent   = curr;
                    }
                }
            }

            marked_edges.clear();
            if (parent_ptrs != nullptr)
                setParentPointers(memory, goal_parent, s_x, s_y, e_x, e_y, parent_ptrs);
            return getPath(memory, goal_parent, s_x, s_y, e_x, e_y);
        }



        Path Algorithm::getPath(const Memory& memory, VertexID goal_parent, const int s_x,
                                const int s_y, const int e_x, const int e_y) const
        {
            Path path;

            // no path found.
            if (goal_parent == NO_PARENT)
                return path;

            // If the last vertex is not equal to the goal vertex.
            if (!(e_x == graph.vertices[goal_parent].x &&
                  e_y == graph.vertices[goal_parent].y))
            {
                path.push_back(GridVertex(e_x, e_y));
            }

            const int LEVEL_W = graph.LEVEL_W;
            VertexID prev     = NO_PARENT;
            VertexID curr     = goal_parent;
            // Assumption: first edge from goal is not a skip-edge.
            // Loop invariant: prev is nonnegative.
            while (curr != NO_PARENT)
            {
                if (curr >= 0)
                {
                    // Normal parent
                    path.push_back(graph.vertices[curr]);
                    prev = curr;
                    curr = memory.parent(curr);
                }
                else
                {
                    // Skip-edge parent
                    curr              = negatePar(curr);
                    const auto& edges = graph.edges;

                    // Set first edge.
                    EdgeID curr_edge;
                    {
                        const auto& edge_list = graph.edge_lists[prev];
                        for (size_t i = 0; i < edge_list.size(); ++i)
                        {
                            const EdgeID id = edge_list[i];
                            if (edges[id].dest_vertex != curr)
                                continue;
                            curr_edge = id;
                            break;
                        }
                    }

                    // Follow level-W edges until you reach then ext skip-vertex.
                    path.push_back(graph.vertices[curr]);
                    while (!graph.isSkipVertex(edges[curr_edge].dest_vertex))
                    {
                        const auto& taut_outgoing_edges =
                            edges[curr_edge].taut_outgoing_edges;

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
                        path.push_back(graph.vertices[edges[curr_edge].dest_vertex]);
                        // This should not infinite loop done correctly.
                    }

                    prev = edges[curr_edge].dest_vertex;
                    curr = memory.parent(prev);
                }
            }

            // If Start vertex is not equal to the first vertex.
            if (s_x != path.back().x || s_y != path.back().y)
            {
                path.push_back(GridVertex(s_x, s_y));
            }

            std::reverse(path.begin(), path.end());
            return path;
        }

        void Algorithm::setParentPointers(const Memory& memory, VertexID goal_parent,
                                          int s_x, int s_y, int e_x, int e_y,
                                          ParentPtrs* parent_ptrs) const
        {
            parent_ptrs->goal        = GridVertex(e_x, e_y);
            parent_ptrs->goal_parent = (goal_parent == NO_PARENT)
                                           ? parent_ptrs->goal
                                           : graph.vertices[restorePar(goal_parent)];

            std::vector<GridVertex>& current = parent_ptrs->current;
            std::vector<GridVertex>& parent  = parent_ptrs->parent;

            current.clear();
            parent.clear();
            for (size_t i = 0; i < memory.n_nodes; ++i)
            {
                if (memory.distance(i) == POS_INF)
                    continue;

                current.push_back(graph.vertices[i]);
                if (memory.parent(i) != NO_PARENT)
                {
                    parent.push_back(graph.vertices[restorePar(memory.parent(i))]);
                }
                else
                {
                    parent.push_back(GridVertex(s_x, s_y));
                }
            }
        }

        size_t Algorithm::nVertices() const
        {
            return graph.vertices.size();
        }

        size_t Algorithm::nEdges() const
        {
            return graph.edges.size();
        }
    }  // namespace Enlsvg
}  // namespace Pathfinding
