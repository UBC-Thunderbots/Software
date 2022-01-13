#ifndef _PATHFINDING_DATA_TYPES_H_
#define _PATHFINDING_DATA_TYPES_H_

#include <cstddef>
#include <vector>

namespace Pathfinding {

struct GridVertex {
    int x;
    int y;

    GridVertex() {}
    GridVertex(int x, int y): x(x), y(y) {}
    
    inline bool operator==(const GridVertex &other)
    {
        return (x == other.x) && (y == other.y);
    }
    
    inline bool operator!=(const GridVertex &other)
    {
        return (x != other.x) || (y != other.y);
    }
};

typedef std::vector<GridVertex> Path;

struct ParentPtrs {
    std::vector<GridVertex> current;
    std::vector<GridVertex> parent;
    GridVertex goal;
    GridVertex goalParent;
};

typedef int VertexID;
typedef size_t EdgeID;

}

#endif