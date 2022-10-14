#ifndef _ENLSVG_ALGORITHM_H_
#define _ENLSVG_ALGORITHM_H_

#include <cmath>
#include <limits>
#include <vector>

#include "enlsvg_graph.h"
#include "grid.h"
#include "indirect_heap.h"
#include "line_of_sight_scanner.h"
#include "pathfinding_data_types.h"

namespace Pathfinding
{
    class Grid;

    namespace Enlsvg
    {
        // NO_PARENT should be positive to be immune to restorePar
        static constexpr VertexID NO_PARENT = std::numeric_limits<VertexID>::max();
        class Algorithm;

        struct AStarData
        {
            bool visited            = false;
            double edgeWeightToGoal = -1.0;
            VertexID parent         = NO_PARENT;
            double distance         = POS_INF;

            AStarData()
                : visited(false),
                  edgeWeightToGoal(-1.0),
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
            const size_t nEdges;
            const size_t nNodes;
            const AStarData def;  // default values
            std::vector<AStarData> nodes;
            std::vector<int> ticketCheck;
            int ticketNumber;

            ScannerStacks scannerStacks;
            MarkedEdges markedEdges;
            IndirectHeap pq;


            void initialise()
            {
                if (ticketNumber != -1)
                {
                    ++ticketNumber;
                }
                else
                {
                    std::fill(ticketCheck.begin(), ticketCheck.end(), 0);
                    ticketNumber = 1;
                }
            }

            bool validate(const VisibilityGraph& graph)
            {
                return nNodes == graph.vertices.size() && nEdges == graph.edges.size();
            }

            inline bool visited(size_t index) const
            {
                return ticketCheck[index] == ticketNumber ? nodes[index].visited
                                                          : def.visited;
            }
            inline double edgeWeightToGoal(size_t index) const
            {
                return ticketCheck[index] == ticketNumber ? nodes[index].edgeWeightToGoal
                                                          : def.edgeWeightToGoal;
            }
            inline VertexID parent(size_t index) const
            {
                return ticketCheck[index] == ticketNumber ? nodes[index].parent
                                                          : def.parent;
            }
            inline double distance(size_t index) const
            {
                return ticketCheck[index] == ticketNumber ? nodes[index].distance
                                                          : def.distance;
            }

            inline void updateData(size_t index)
            {
                if (ticketCheck[index] != ticketNumber)
                {
                    nodes[index]       = def;
                    ticketCheck[index] = ticketNumber;
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
                nodes[index].edgeWeightToGoal = value;
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

            Path computePath(Memory& memory, const int sx, const int sy, const int ex,
                             const int ey, ParentPtrs* parentPtrs = nullptr) const;
            Path computeSVGPath(Memory& memory, const int sx, const int sy, const int ex,
                                const int ey, ParentPtrs* parentPtrs = nullptr) const;

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

            Path getPath(const Memory& memory, VertexID goalParent, const int sx,
                         const int sy, const int ex, const int ey) const;
            void setParentPointers(const Memory& memory, VertexID goalParent, int sx,
                                   int sy, int ex, int ey, ParentPtrs* parentPtrs) const;
        };
    }  // namespace Enlsvg
}  // namespace Pathfinding

#endif
