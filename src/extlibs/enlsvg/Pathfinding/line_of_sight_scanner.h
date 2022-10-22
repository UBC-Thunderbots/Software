#ifndef _LINE_OF_SIGHT_SCANNER_H_
#define _LINE_OF_SIGHT_SCANNER_H_

#include <algorithm>

#include "fraction.h"
#include "pathfinding_data_types.h"

namespace Pathfinding
{
    class Grid;

    // typedef double Rational;
    typedef Fraction Rational;

    struct ScanInterval
    {
        const int y;
        const Rational x_left;
        const Rational x_right;
        const bool left_inclusive;
        const bool right_inclusive;

        ScanInterval(int y, const Rational x_left, const Rational x_right,
                     bool left_inclusive, bool right_inclusive)
            : y(y),
              x_left(x_left),
              x_right(x_right),
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

        void computeAllDirNeighbours(ScannerStacks& data, int s_x, int s_y) const;
        void computeTautDirNeighbours(ScannerStacks& data, int s_x, int s_y) const;
        void computeExtents();

        inline int leftUpExtent(int x_left, int y) const
        {
            return x_left > size_x ? size_x
                                   : left_down_extents[(y + 1) * extents_size_x + x_left];
        }

        inline int leftDownExtent(int x_left, int y) const
        {
            return x_left > size_x ? size_x
                                   : left_down_extents[y * extents_size_x + x_left];
        }

        inline int leftAnyExtent(int x_left, int y) const
        {
            return std::max(left_down_extents[y * extents_size_x + x_left],
                            left_down_extents[(y + 1) * extents_size_x + x_left]);
        }

        inline int rightUpExtent(int x_right, int y) const
        {
            return x_right < 0 ? 0
                               : right_down_extents[(y + 1) * extents_size_x + x_right];
        }

        inline int rightDownExtent(int x_right, int y) const
        {
            return x_right < 0 ? 0 : right_down_extents[y * extents_size_x + x_right];
        }

        inline int rightAnyExtent(int x_right, int y) const
        {
            return std::min(right_down_extents[y * extents_size_x + x_right],
                            right_down_extents[(y + 1) * extents_size_x + x_right]);
        }

       private:
        const Grid& grid;
        const int size_x;
        const int size_y;
        const int extents_size_x;

        std::vector<int> right_down_extents;
        std::vector<int> left_down_extents;

        void generateTautDirectionStartingStates(ScannerStacks& data, int s_x,
                                                 int s_y) const;
        void generateAllDirectionStartingStates(ScannerStacks& data, int s_x,
                                                int s_y) const;
        void exploreStates(ScannerStacks& data, int s_x, int s_y) const;
        void generateUpwards(ScannerStacks& data, Rational left_bound,
                             Rational right_bound, int s_x, int s_y, int curr_y,
                             bool left_inclusive, bool right_inclusive) const;
        void generateDownwards(ScannerStacks& data, Rational left_bound,
                               Rational right_bound, int s_x, int s_y, int curr_y,
                               bool left_inclusive, bool right_inclusive) const;
        void generateAndSplitIntervals(ScannerStacks& data, int check_y, int new_y,
                                       int s_x, int s_y, Rational left_bound,
                                       Rational right_bound, bool left_inclusive,
                                       bool right_inclusive) const;
    };

}  // namespace Pathfinding

#endif
