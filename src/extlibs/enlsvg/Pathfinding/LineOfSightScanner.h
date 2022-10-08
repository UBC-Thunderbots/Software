#ifndef _LINE_OF_SIGHT_SCANNER_H_
#define _LINE_OF_SIGHT_SCANNER_H_

#include <algorithm>

#include "Fraction.h"
#include "PathfindingDataTypes.h"

namespace Pathfinding
{
    class Grid;

    // typedef double Rational;
    typedef Fraction Rational;

    struct ScanInterval
    {
        const int y;
        const Rational x_L;
        const Rational x_R;
        const bool left_inclusive;
        const bool right_inclusive;

        ScanInterval(int y, const Rational x_L, const Rational x_R, bool left_inclusive,
                     bool right_inclusive)
            : y(y),
              x_L(x_L),
              x_R(x_R),
              left_inclusive(left_inclusive),
              right_inclusive(right_inclusive)
        {
        }
    };

    struct ScannerStacks
    {
        std::vector<GridVertex> neighbours;
        std::vector<ScanInterval> interval_stack;

        inline void clear()
        {
            neighbours.clear();
            interval_stack.clear();
        }

        inline void addSuccessor(GridVertex grid_vertex)
        {
            neighbours.push_back(grid_vertex);
        }

        inline void stackPush(ScanInterval interval)
        {
            interval_stack.push_back(interval);
        }

        inline ScanInterval stackPop()
        {
            ScanInterval last = interval_stack.back();
            interval_stack.pop_back();
            return last;
        }
    };

    class LineOfSightScanner
    {
       public:
        LineOfSightScanner(const Grid& grid);

        void computeAllDirNeighbours(ScannerStacks& data, int sx, int sy) const;
        void computeTautDirNeighbours(ScannerStacks& data, int sx, int sy) const;
        void computeExtents();

        inline int leftUpExtent(int x_L, int y) const
        {
            return x_L > size_x ? size_x : left_down_extents[(y + 1) * extents_size_x + x_L];
        }

        inline int leftDownExtent(int xL, int y) const
        {
            return xL > size_x ? size_x : left_down_extents[y * extents_size_x + xL];
        }

        inline int leftAnyExtent(int xL, int y) const
        {
            return std::max(left_down_extents[y * extents_size_x + xL],
                            left_down_extents[(y + 1) * extents_size_x + xL]);
        }

        inline int rightUpExtent(int xR, int y) const
        {
            return xR < 0 ? 0 : right_down_extents[(y + 1) * extents_size_x + xR];
        }

        inline int rightDownExtent(int xR, int y) const
        {
            return xR < 0 ? 0 : right_down_extents[y * extents_size_x + xR];
        }

        inline int rightAnyExtent(int xR, int y) const
        {
            return std::min(right_down_extents[y * extents_size_x + xR],
                            right_down_extents[(y + 1) * extents_size_x + xR]);
        }

       private:
        const Grid& grid;
        const int size_x;
        const int sizeY;
        const int extents_size_x;

        std::vector<int> right_down_extents;
        std::vector<int> left_down_extents;

        void generateTautDirectionStartingStates(ScannerStacks& data, int sx,
                                                 int sy) const;
        void generateAllDirectionStartingStates(ScannerStacks& data, int sx,
                                                int sy) const;
        void exploreStates(ScannerStacks& data, int sx, int sy) const;
        void generateUpwards(ScannerStacks& data, Rational left_bound, Rational right_bound,
                             int sx, int sy, int curr_y, bool left_inclusive,
                             bool right_inclusive) const;
        void generateDownwards(ScannerStacks& data, Rational left_bound,
                               Rational right_bound, int sx, int sy, int curr_y,
                               bool left_inclusive, bool right_inclusive) const;
        void generateAndSplitIntervals(ScannerStacks& data, int check_y, int new_y, int sx,
                                       int sy, Rational left_bound, Rational right_bound,
                                       bool left_inclusive, bool right_inclusive) const;
    };

}  // namespace Pathfinding

#endif
