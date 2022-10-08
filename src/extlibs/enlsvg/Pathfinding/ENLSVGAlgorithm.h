#ifndef _ENLSVG_ALGORITHM_H_
#define _ENLSVG_ALGORITHM_H_

#include <cmath>
#include <limits>
#include <vector>

#include "ENLSVGGraph.h"
#include "Grid.h"
#include "IndirectHeap.h"
#include "LineOfSightScanner.h"
#include "PathfindingDataTypes.h"

namespace Pathfinding
{
    class Grid;

    namespace ENLSVG
    {
        // NO_PARENT should be positive to be immune to restorePar
        static constexpr VertexID NO_PARENT = std::numeric_limits<VertexID>::max();
        class Algorithm;

        struct AStarData
        {
            bool visited               = false;
            double edge_weight_to_goal = -1.0;
            VertexID parent            = NO_PARENT;
            double distance            = POS_INF;

            AStarData()
                : visited(false),
                  edge_weight_to_goal(-1.0),
                  parent(NO_PARENT),
                  distance(POS_INF)
            {
            }
        };

        class Memory
        {
            friend class Algorithm;

           public:
            Memory(const Algorithm& algo);

           private:
            const size_t n_edges;
            const size_t n_nodes;
            const AStarData def;  // default values
            std::vector<AStarData> nodes;
            std::vector<int> ticket_check;
            int ticket_number;

            ScannerStacks scanner_stacks;
            MarkedEdges marked_edges;
            IndirectHeap pq;


            void initialise()
            {
                if (ticket_number != -1)
                {
                    ++ticket_number;
                }
                else
                {
                    std::fill(ticket_check.begin(), ticket_check.end(), 0);
                    ticket_number = 1;
                }
            }

            bool validate(const VisibilityGraph& graph)
            {
                return n_nodes == graph.vertices.size() && n_edges == graph.edges.size();
            }

            inline bool visited(size_t index) const
            {
                return ticket_check[index] == ticket_number ? nodes[index].visited
                                                            : def.visited;
            }
            inline double edgeWeightToGoal(size_t index) const
            {
                return ticket_check[index] == ticket_number
                           ? nodes[index].edge_weight_to_goal
                           : def.edge_weight_to_goal;
            }
            inline VertexID parent(size_t index) const
            {
                return ticket_check[index] == ticket_number ? nodes[index].parent
                                                            : def.parent;
            }
            inline double distance(size_t index) const
            {
                return ticket_check[index] == ticket_number ? nodes[index].distance
                                                            : def.distance;
            }

            inline void updateData(size_t index)
            {
                if (ticket_check[index] != ticket_number)
                {
                    nodes[index]        = def;
                    ticket_check[index] = ticket_number;
                }
            }

            inline void setVisited(size_t index, bool value)
            {
                updateData(index);
                nodes[index].visited = value;
            }

            inline void setEdgeWeightToGoal(size_t index, double value)
            {
                updateData(index);
                nodes[index].edge_weight_to_goal = value;
            }

            inline void setParent(size_t index, VertexID value)
            {
                updateData(index);
                nodes[index].parent = value;
            }

            inline void setDistance(size_t index, double value)
            {
                updateData(index);
                nodes[index].distance = value;
            }
        };

        class Algorithm
        {
           private:
            const Grid& grid;
            const LineOfSightScanner scanner;
            const VisibilityGraph
                graph;  // Note: This must be defined after scanner and grid.

           public:
            Algorithm(const Grid& grid);

            Path computePath(Memory& memory, const int s_x, const int s_y, const int e_x,
                             const int e_y, ParentPtrs* parent_ptrs = nullptr) const;
            Path computeSVGPath(Memory& memory, const int s_x, const int s_y,
                                const int e_x, const int e_y,
                                ParentPtrs* parent_ptrs = nullptr) const;

            size_t nVertices() const;
            size_t nEdges() const;

            void printStatistics() const
            {
                graph.printStatistics();
            }

           private:
            inline double heuristic(int index, int ex, int ey) const
            {
                int dx = graph.vertices[index].x - ex;
                int dy = graph.vertices[index].y - ey;
                return sqrt(dx * dx + dy * dy);
            }

            inline bool isTaut(int parent, int curr, int next) const
            {
                if (parent == NO_PARENT)
                    return true;
                const std::vector<GridVertex>& vertices = graph.vertices;
                int x1                                  = vertices[parent].x;
                int y1                                  = vertices[parent].y;
                int x2                                  = vertices[curr].x;
                int y2                                  = vertices[curr].y;
                int x3                                  = vertices[next].x;
                int y3                                  = vertices[next].y;

                return grid.isTaut(x1, y1, x2, y2, x3, y3);
            }

            Path getPath(const Memory& memory, VertexID goal_parent, const int s_x,
                         const int s_y, const int e_x, const int e_y) const;
            void setParentPointers(const Memory& memory, VertexID goal_parent, int s_x,
                                   int s_y, int e_x, int e_y,
                                   ParentPtrs* parent_ptrs) const;
        };
    }  // namespace ENLSVG
}  // namespace Pathfinding

#endif
