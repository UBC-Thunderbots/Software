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
        const Rational xL;
        const Rational xR;
        const bool leftInclusive;
        const bool rightInclusive;

        ScanInterval(int y, const Rational xL, const Rational xR, bool leftInclusive,
                     bool rightInclusive)
            : y(y),
              xL(xL),
              xR(xR),
              leftInclusive(leftInclusive),
              rightInclusive(rightInclusive)
        {
        }
    };

    struct ScannerStacks
    {
        std::vector<GridVertex> neighbours;
        std::vector<ScanInterval> intervalStack;

        inline void clear()
        {
            neighbours.clear();
            intervalStack.clear();
        }

        inline void addSuccessor(GridVertex gridVertex)
        {
            neighbours.push_back(gridVertex);
        }

        inline void stackPush(ScanInterval interval)
        {
            intervalStack.push_back(interval);
        }

        inline ScanInterval stackPop()
        {
            ScanInterval last = intervalStack.back();
            intervalStack.pop_back();
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

        inline int leftUpExtent(int xL, int y) const
        {
            return xL > sizeX ? sizeX : leftDownExtents[(y + 1) * extentsSizeX + xL];
        }

        inline int leftDownExtent(int xL, int y) const
        {
            return xL > sizeX ? sizeX : leftDownExtents[y * extentsSizeX + xL];
        }

        inline int leftAnyExtent(int xL, int y) const
        {
            return std::max(leftDownExtents[y * extentsSizeX + xL],
                            leftDownExtents[(y + 1) * extentsSizeX + xL]);
        }

        inline int rightUpExtent(int xR, int y) const
        {
            return xR < 0 ? 0 : rightDownExtents[(y + 1) * extentsSizeX + xR];
        }

        inline int rightDownExtent(int xR, int y) const
        {
            return xR < 0 ? 0 : rightDownExtents[y * extentsSizeX + xR];
        }

        inline int rightAnyExtent(int xR, int y) const
        {
            return std::min(rightDownExtents[y * extentsSizeX + xR],
                            rightDownExtents[(y + 1) * extentsSizeX + xR]);
        }

       private:
        const Grid& grid;
        const int sizeX;
        const int sizeY;
        const int extentsSizeX;

        std::vector<int> rightDownExtents;
        std::vector<int> leftDownExtents;

        void generateTautDirectionStartingStates(ScannerStacks& data, int sx,
                                                 int sy) const;
        void generateAllDirectionStartingStates(ScannerStacks& data, int sx,
                                                int sy) const;
        void exploreStates(ScannerStacks& data, int sx, int sy) const;
        void generateUpwards(ScannerStacks& data, Rational leftBound, Rational rightBound,
                             int sx, int sy, int currY, bool leftInclusive,
                             bool rightInclusive) const;
        void generateDownwards(ScannerStacks& data, Rational leftBound,
                               Rational rightBound, int sx, int sy, int currY,
                               bool leftInclusive, bool rightInclusive) const;
        void generateAndSplitIntervals(ScannerStacks& data, int checkY, int newY, int sx,
                                       int sy, Rational leftBound, Rational rightBound,
                                       bool leftInclusive, bool rightInclusive) const;
    };

}  // namespace Pathfinding

#endif
