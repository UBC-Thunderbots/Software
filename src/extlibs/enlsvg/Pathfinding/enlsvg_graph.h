#ifndef _ENLSVG_GRAPH_H_
#define _ENLSVG_GRAPH_H_

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "pathfinding_data_types.h"

namespace Pathfinding
{
    class Grid;
    class LineOfSightScanner;

    namespace Enlsvg
    {
        struct EdgeData
        {
            const VertexID source_vertex;
            const VertexID dest_vertex;
            int level;
            std::vector<EdgeID> taut_outgoing_edges;

            EdgeData(VertexID source_vertex, VertexID dest_vertex, int level)
                : source_vertex(source_vertex), dest_vertex(dest_vertex), level(level)
            {
            }
        };

        struct SkipEdge
        {
            const VertexID next;
            const double weight;
            const VertexID immediate_next;  // immediate vertex just after current
            const VertexID immediate_last;  // immediate vertex just before next.

            SkipEdge(VertexID next, double weight, VertexID im_next, VertexID im_last)
                : next(next),
                  weight(weight),
                  immediate_next(im_next),
                  immediate_last(im_last)
            {
            }
        };

        struct MarkedEdges
        {
            std::vector<bool> is_marked;
            std::vector<EdgeID> marked_indexes;

            MarkedEdges(size_t n_edges)
            {
                is_marked.resize(n_edges, false);
            }

            inline void mark(EdgeID index)
            {
                is_marked[index] = true;
                marked_indexes.push_back(index);
            }

            inline void clear()
            {
                for (size_t i = 0; i < marked_indexes.size(); ++i)
                {
                    is_marked[marked_indexes[i]] = false;
                }
                marked_indexes.clear();
            }
        };

        class VisibilityGraph
        {
           public:
            VisibilityGraph(const Grid& grid, const LineOfSightScanner& scanner);
            void markEdgesFrom(MarkedEdges& marked_edges, const int s_x, const int s_y,
                               const std::vector<GridVertex>& neighbours) const;
            void markBothWays(MarkedEdges& marked_edges) const;
            inline bool isSkipVertex(VertexID vertex_id) const
            {
                return skip_edges[vertex_id].size() > 0;
            }
            inline VertexID nodeIndex(int x, int y) const
            {
                return node_indexes[y * node_indexes_size_x + x];
            }
            inline double weight(const EdgeData& edge) const
            {
                const GridVertex& u = vertices[edge.source_vertex];
                const GridVertex& v = vertices[edge.dest_vertex];
                int dx              = v.x - u.x;
                int dy              = v.y - u.y;
                return sqrt(dx * dx + dy * dy);
            }
            inline double weight(EdgeID edge_id) const
            {
                return weight(edges[edge_id]);
            }

            void printStatistics() const;

            static constexpr int LEVEL_W = std::numeric_limits<VertexID>::max();
            const int size_x;
            const int size_y;
            const int node_indexes_size_x;

            // Indexed by VertexID
            std::vector<GridVertex> vertices;
            std::vector<std::vector<EdgeID>> edge_lists;  // points to edge indexes.
            std::vector<std::vector<SkipEdge>> skip_edges;

            // Indexed by grid coordinates.
            std::vector<VertexID> node_indexes;

            // Indexed by EdgeID
            std::vector<EdgeData> edges;

           private:
            const Grid& grid;
            const LineOfSightScanner& scanner;

            inline EdgeID opposite(EdgeID edge_id) const
            {
                return (edge_id % 2 == 0) ? edge_id + 1 : edge_id - 1;
            }

            void connectEdge(int i, int j, int x_i, int y_i, int x_j, int y_j);
            void buildHierarchy();
            void computeAllEdgeLevels();
            void setupSkipEdges();
            void followLevelWPathToNextSkipVertex(
                EdgeID first_edge, double& total_weight, VertexID& next_vertex,
                VertexID& immediate_next, VertexID& immediate_last,
                const std::vector<bool>& is_skip_vertex) const;
        };
    }  // namespace Enlsvg
}  // namespace Pathfinding


#endif
