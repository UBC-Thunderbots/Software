#include "extlibs/enlsvg/grid.h"

#include <iostream>

namespace Pathfinding
{
    Grid::Grid(int size_x, int size_y)
        : size_x(size_x), size_y(size_y), total_size(size_x * size_y)
    {
        blocked.resize(size_x * size_y);
    }

    void Grid::printGrid() const
    {
        for (int y = 0; y < size_y; ++y)
        {
            for (int x = 0; x < size_x; ++x)
            {
                std::cout << (isBlockedRaw(x, y) ? "X" : " ");
            }
            std::cout << std::endl;
        }
    }

}  // namespace Pathfinding
