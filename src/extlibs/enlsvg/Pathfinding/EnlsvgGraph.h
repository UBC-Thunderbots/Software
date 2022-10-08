#ifndef _ENLSVG_GRAPH_H_
#define _ENLSVG_GRAPH_H_

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "PathfindingDataTypes.h"

namespace Pathfinding
{
    class Grid;
    class LineOfSightScanner;

    namespace Enlsvg
    {
        struct EdgeData
        {
            const VertexID sourceVertex;
            const VertexID destVertex;
            int level;
            std::vector<EdgeID> tautOutgoingEdges;

            EdgeData(VertexID sourceVertex, VertexID destVertex, int level)
                : sourceVertex(sourceVertex), destVertex(destVertex), level(level)
            {
            }
        };

        struct SkipEdge
        {
            const VertexID next;
            const double weight;
            const VertexID immediateNext;  // immediate vertex just after current
            const VertexID immediateLast;  // immediate vertex just before next.

            SkipEdge(VertexID next, double weight, VertexID imNext, VertexID imLast)
                : next(next), weight(weight), immediateNext(imNext), immediateLast(imLast)
            {
            }
        };

        struct MarkedEdges
        {
            std::vector<bool> isMarked;
            std::vector<EdgeID> markedIndexes;

            MarkedEdges(size_t nEdges)
            {
                isMarked.resize(nEdges, false);
            }

            inline void mark(EdgeID index)
            {
                isMarked[index] = true;
                markedIndexes.push_back(index);
            }

            inline void clear()
            {
                for (size_t i = 0; i < markedIndexes.size(); ++i)
                {
                    isMarked[markedIndexes[i]] = false;
                }
                markedIndexes.clear();
            }
        };

        class VisibilityGraph
        {
           public:
            VisibilityGraph(const Grid& grid, const LineOfSightScanner& scanner);
            void markEdgesFrom(MarkedEdges& markedEdges, const int sx, const int sy,
                               const std::vector<GridVertex>& neighbours) const;
            void markBothWays(MarkedEdges& markedEdges) const;
            inline bool isSkipVertex(VertexID vertexID) const
            {
                return skipEdges[vertexID].size() > 0;
            }
            inline VertexID nodeIndex(int x, int y) const
            {
                return nodeIndexes[y * nodeIndexesSizeX + x];
            }
            inline double weight(const EdgeData& edge) const
            {
                const GridVertex& u = vertices[edge.sourceVertex];
                const GridVertex& v = vertices[edge.destVertex];
                int dx              = v.x - u.x;
                int dy              = v.y - u.y;
                return sqrt(dx * dx + dy * dy);
            }
            inline double weight(EdgeID edgeId) const
            {
                return weight(edges[edgeId]);
            }

            void printStatistics() const;

            static constexpr int LEVEL_W = std::numeric_limits<VertexID>::max();
            const int sizeX;
            const int sizeY;
            const int nodeIndexesSizeX;

            // Indexed by VertexID
            std::vector<GridVertex> vertices;
            std::vector<std::vector<EdgeID>> edgeLists;  // points to edge indexes.
            std::vector<std::vector<SkipEdge>> skipEdges;

            // Indexed by grid coordinates.
            std::vector<VertexID> nodeIndexes;

            // Indexed by EdgeID
            std::vector<EdgeData> edges;

           private:
            const Grid& grid;
            const LineOfSightScanner& scanner;

            inline EdgeID opposite(EdgeID edgeID) const
            {
                return (edgeID % 2 == 0) ? edgeID + 1 : edgeID - 1;
            }

            void connectEdge(int i, int j, int xi, int yi, int xj, int yj);
            void buildHierarchy();
            void computeAllEdgeLevels();
            void setupSkipEdges();
            void followLevelWPathToNextSkipVertex(
                EdgeID firstEdge, double& totalWeight, VertexID& nextVertex,
                VertexID& immediateNext, VertexID& immediateLast,
                const std::vector<bool>& isSkipVertex) const;
        };
    }  // namespace Enlsvg
}  // namespace Pathfinding


#endif
