#include "extlibs/enlsvg/Pathfinding/line_of_sight_scanner.h"

#include "extlibs/enlsvg/Pathfinding/grid.h"

namespace Pathfinding
{
    LineOfSightScanner::LineOfSightScanner(const Grid& grid)
        : grid(grid), sizeX(grid.sizeX), sizeY(grid.sizeY), extentsSizeX(grid.sizeX + 1)
    {
        computeExtents();
    }

    void LineOfSightScanner::computeAllDirNeighbours(ScannerStacks& data, int sx,
                                                     int sy) const
    {
        data.clear();

        generateAllDirectionStartingStates(data, sx, sy);
        exploreStates(data, sx, sy);
    }

    void LineOfSightScanner::computeTautDirNeighbours(ScannerStacks& data, int sx,
                                                      int sy) const
    {
        data.clear();

        generateTautDirectionStartingStates(data, sx, sy);
        exploreStates(data, sx, sy);
    }


    void LineOfSightScanner::computeExtents()
    {
        rightDownExtents.resize(extentsSizeX * (sizeY + 2));
        leftDownExtents.resize(extentsSizeX * (sizeY + 2));

        for (int y = 0; y < sizeY + 2; ++y)
        {
            bool lastIsBlocked = true;
            int lastX          = -1;
            for (int x = 0; x <= sizeX; ++x)
            {
                leftDownExtents[y * extentsSizeX + x] = lastX;
                if (grid.isBlocked(x, y - 1) != lastIsBlocked)
                {
                    lastX         = x;
                    lastIsBlocked = !lastIsBlocked;
                }
            }
            lastIsBlocked = true;
            lastX         = sizeX + 1;
            for (int x = sizeX; x >= 0; --x)
            {
                rightDownExtents[y * extentsSizeX + x] = lastX;
                if (grid.isBlocked(x - 1, y - 1) != lastIsBlocked)
                {
                    lastX         = x;
                    lastIsBlocked = !lastIsBlocked;
                }
            }
        }
    }



    /**
     * Assumption: We are at an outer corner. One of six cases:
     *   BR        BL        TR        TL       TRBL      TLBR
     * XXX|         |XXX      :         :         |XXX   XXX|
     * XXX|...   ...|XXX   ___:...   ...:___   ___|XXX   XXX|___
     *    :         :      XXX|         |XXX   XXX|         |XXX
     *    :         :      XXX|         |XXX   XXX|         |XXX
     */
    void LineOfSightScanner::generateTautDirectionStartingStates(ScannerStacks& data,
                                                                 int sx, int sy) const
    {
        bool bottomLeftOfBlocked  = grid.bottomLeftOfBlockedTile(sx, sy);
        bool bottomRightOfBlocked = grid.bottomRightOfBlockedTile(sx, sy);
        bool topLeftOfBlocked     = grid.topLeftOfBlockedTile(sx, sy);
        bool topRightOfBlocked    = grid.topRightOfBlockedTile(sx, sy);

        // Generate up-left direction
        if (topRightOfBlocked || bottomLeftOfBlocked)
        {
            Fraction leftExtent(leftUpExtent(sx, sy));
            Fraction rightExtent(sx);

            generateUpwards(data, leftExtent, rightExtent, sx, sy, sy, true, true);
        }

        // Generate up-right direction
        if (bottomRightOfBlocked || topLeftOfBlocked)
        {
            Fraction leftExtent(sx);
            Fraction rightExtent(rightUpExtent(sx, sy));

            generateUpwards(data, leftExtent, rightExtent, sx, sy, sy, true, true);
        }

        // Generate down-left direction
        if (bottomRightOfBlocked || topLeftOfBlocked)
        {
            Fraction leftExtent(leftDownExtent(sx, sy));
            Fraction rightExtent(sx);

            generateDownwards(data, leftExtent, rightExtent, sx, sy, sy, true, true);
        }

        // Generate down-right direction
        if (topRightOfBlocked || bottomLeftOfBlocked)
        {
            Fraction leftExtent(sx);
            Fraction rightExtent(rightDownExtent(sx, sy));

            generateDownwards(data, leftExtent, rightExtent, sx, sy, sy, true, true);
        }

        // Search leftwards
        if (!topRightOfBlocked || !bottomRightOfBlocked)
        {
            int x = leftAnyExtent(sx, sy);
            int y = sy;
            if (!(grid.topRightOfBlockedTile(x, y) &&
                  grid.bottomRightOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }

        // Search rightwards
        if (!topLeftOfBlocked || !bottomLeftOfBlocked)
        {
            int x = rightAnyExtent(sx, sy);
            int y = sy;
            if (!(grid.topLeftOfBlockedTile(x, y) && grid.bottomLeftOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }
    }

    void LineOfSightScanner::generateAllDirectionStartingStates(ScannerStacks& data,
                                                                int sx, int sy) const
    {
        bool bottomLeftOfBlocked  = grid.bottomLeftOfBlockedTile(sx, sy);
        bool bottomRightOfBlocked = grid.bottomRightOfBlockedTile(sx, sy);
        bool topLeftOfBlocked     = grid.topLeftOfBlockedTile(sx, sy);
        bool topRightOfBlocked    = grid.topRightOfBlockedTile(sx, sy);

        // Generate up
        if (!bottomLeftOfBlocked || !bottomRightOfBlocked)
        {
            Fraction leftExtent, rightExtent;

            if (bottomLeftOfBlocked)
            {
                // Explore up-left
                leftExtent  = Fraction(leftUpExtent(sx, sy));
                rightExtent = Fraction(sx);
            }
            else if (bottomRightOfBlocked)
            {
                // Explore up-right
                leftExtent  = Fraction(sx);
                rightExtent = Fraction(rightUpExtent(sx, sy));
            }
            else
            {
                // Explore up-left-right
                leftExtent  = Fraction(leftUpExtent(sx, sy));
                rightExtent = Fraction(rightUpExtent(sx, sy));
            }

            generateUpwards(data, leftExtent, rightExtent, sx, sy, sy, true, true);
        }

        // Generate down
        if (!topLeftOfBlocked || !topRightOfBlocked)
        {
            Fraction leftExtent, rightExtent;

            if (topLeftOfBlocked)
            {
                // Explore down-left
                leftExtent  = Fraction(leftDownExtent(sx, sy));
                rightExtent = Fraction(sx);
            }
            else if (topRightOfBlocked)
            {
                // Explore down-right
                leftExtent  = Fraction(sx);
                rightExtent = Fraction(rightDownExtent(sx, sy));
            }
            else
            {
                // Explore down-left-right
                leftExtent  = Fraction(leftDownExtent(sx, sy));
                rightExtent = Fraction(rightDownExtent(sx, sy));
            }

            generateDownwards(data, leftExtent, rightExtent, sx, sy, sy, true, true);
        }

        // Search leftwards
        if (!topRightOfBlocked || !bottomRightOfBlocked)
        {
            int x = leftAnyExtent(sx, sy);
            int y = sy;
            if (!(grid.topRightOfBlockedTile(x, y) &&
                  grid.bottomRightOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }

        // Search rightwards
        if (!topLeftOfBlocked || !bottomLeftOfBlocked)
        {
            int x = rightAnyExtent(sx, sy);
            int y = sy;
            if (!(grid.topLeftOfBlockedTile(x, y) && grid.bottomLeftOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }
    }

    void LineOfSightScanner::exploreStates(ScannerStacks& data, int sx, int sy) const
    {
        while (data.intervalStack.size() > 0)
        {
            ScanInterval currState = data.stackPop();
            bool leftInclusive     = currState.leftInclusive;
            bool rightInclusive    = currState.rightInclusive;

            bool zeroLengthInterval = (currState.xR == currState.xL);
            if (currState.y > sy)
            {
                // Upwards

                // Insert endpoints if integer.
                if (leftInclusive && currState.xL.isWholeNumber())
                {
                    /* The two cases   _
                     *  _             |X|
                     * |X|'.           ,'
                     *      '.       ,'
                     *        B     B
                     */

                    int x                         = currState.xL.n;
                    int y                         = currState.y;
                    bool topRightOfBlockedTile    = grid.topRightOfBlockedTile(x, y);
                    bool bottomRightOfBlockedTile = grid.bottomRightOfBlockedTile(x, y);

                    if (x <= sx && topRightOfBlockedTile && !bottomRightOfBlockedTile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        leftInclusive = false;
                    }
                    else if (sx <= x && bottomRightOfBlockedTile &&
                             !topRightOfBlockedTile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        leftInclusive = false;
                    }
                }
                if (rightInclusive && currState.xR.isWholeNumber())
                {
                    /*   _   The two cases
                     *  |X|             _
                     *  '.           ,'|X|
                     *    '.       ,'
                     *      B     B
                     */

                    int x                        = currState.xR.n;
                    int y                        = currState.y;
                    bool bottomLeftOfBlockedTile = grid.bottomLeftOfBlockedTile(x, y);
                    bool topLeftOfBlockedTile    = grid.topLeftOfBlockedTile(x, y);

                    if (x <= sx && bottomLeftOfBlockedTile && !topLeftOfBlockedTile)
                    {
                        if (leftInclusive || !zeroLengthInterval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            rightInclusive = false;
                        }
                    }
                    else if (sx <= x && topLeftOfBlockedTile && !bottomLeftOfBlockedTile)
                    {
                        if (leftInclusive || !zeroLengthInterval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            rightInclusive = false;
                        }
                    }
                }


                // Generate Upwards
                /*
                 * =======      =====    =====
                 *  \   /       / .'      '. \
                 *   \ /   OR  /.'    OR    '.\
                 *    B       B                B
                 */

                // (Px-Bx)*(Py-By+1)/(Py-By) + Bx
                int dy = currState.y - sy;
                Fraction leftProjection =
                    (currState.xL - sx).multiplyDivide(dy + 1, dy) + sx;

                int leftBound = leftUpExtent(currState.xL.ceil(), currState.y);
                if (currState.xL.isWholeNumber() &&
                    grid.bottomRightOfBlockedTile(currState.xL.n, currState.y))
                    leftBound = currState.xL.n;

                if (leftProjection < leftBound)
                {  // leftProjection < leftBound
                    leftProjection = Fraction(leftBound);
                    leftInclusive  = true;
                }

                // (Px-Bx)*(Py-By+1)/(Py-By) + Bx
                Fraction rightProjection =
                    (currState.xR - sx).multiplyDivide(dy + 1, dy) + sx;

                int rightBound = rightUpExtent(currState.xR.floor(), currState.y);
                if (currState.xR.isWholeNumber() &&
                    grid.bottomLeftOfBlockedTile(currState.xR.n, currState.y))
                    rightBound = currState.xR.n;

                if (rightProjection > rightBound)
                {  // rightBound < rightProjection
                    rightProjection = Fraction(rightBound);
                    rightInclusive  = true;
                }

                // Call Generate
                if (leftInclusive && rightInclusive)
                {
                    if (leftProjection <= rightProjection)
                    {
                        generateUpwards(data, leftProjection, rightProjection, sx, sy,
                                        currState.y, true, true);
                    }
                }
                else if (leftProjection < rightProjection)
                {
                    generateUpwards(data, leftProjection, rightProjection, sx, sy,
                                    currState.y, leftInclusive, rightInclusive);
                }
            }
            else
            {
                // Upwards

                // Insert endpoints if integer.
                if (leftInclusive && currState.xL.isWholeNumber())
                {
                    /* The two cases
                     *        B     B
                     *  _   ,'       '.
                     * |X|.'           '.
                     *                |X|
                     */

                    int x                         = currState.xL.n;
                    int y                         = currState.y;
                    bool bottomRightOfBlockedTile = grid.bottomRightOfBlockedTile(x, y);
                    bool topRightOfBlockedTile    = grid.topRightOfBlockedTile(x, y);

                    if (x <= sx && bottomRightOfBlockedTile && !topRightOfBlockedTile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        leftInclusive = false;
                    }
                    else if (sx <= x && topRightOfBlockedTile &&
                             !bottomRightOfBlockedTile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        leftInclusive = false;
                    }
                }
                if (rightInclusive && currState.xR.isWholeNumber())
                {
                    /*       The two cases
                     *      B     B
                     *    .'       '.   _
                     *  .'           '.|X|
                     *  |X|
                     */

                    int x                        = currState.xR.n;
                    int y                        = currState.y;
                    bool topLeftOfBlockedTile    = grid.topLeftOfBlockedTile(x, y);
                    bool bottomLeftOfBlockedTile = grid.bottomLeftOfBlockedTile(x, y);

                    if (x <= sx && topLeftOfBlockedTile && !bottomLeftOfBlockedTile)
                    {
                        if (leftInclusive || !zeroLengthInterval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            rightInclusive = false;
                        }
                    }
                    else if (sx <= x && bottomLeftOfBlockedTile && !topLeftOfBlockedTile)
                    {
                        if (leftInclusive || !zeroLengthInterval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            rightInclusive = false;
                        }
                    }
                }



                // Generate downwards
                /*
                 *    B       B                B
                 *   / \   OR  \'.    OR    .'/
                 *  /   \       \ '.      .' /
                 * =======      =====    =====
                 */

                // (Px-Bx)*(Py-By+1)/(Py-By) + Bx
                int dy = sy - currState.y;
                Fraction leftProjection =
                    (currState.xL - sx).multiplyDivide(dy + 1, dy) + sx;

                int leftBound = leftDownExtent(currState.xL.ceil(), currState.y);
                if (currState.xL.isWholeNumber() &&
                    grid.topRightOfBlockedTile(currState.xL.n, currState.y))
                    leftBound = currState.xL.n;

                if (leftProjection < leftBound)
                {  // leftProjection < leftBound
                    leftProjection = Fraction(leftBound);
                    leftInclusive  = true;
                }

                // (Px-Bx)*(Py-By+1)/(Py-By) + Bx
                Fraction rightProjection =
                    (currState.xR - sx).multiplyDivide(dy + 1, dy) + sx;

                int rightBound = rightDownExtent(currState.xR.floor(), currState.y);
                if (currState.xR.isWholeNumber() &&
                    grid.topLeftOfBlockedTile(currState.xR.n, currState.y))
                    rightBound = currState.xR.n;

                if (rightProjection > rightBound)
                {  // rightBound < rightProjection
                    rightProjection = Fraction(rightBound);
                    rightInclusive  = true;
                }

                // Call Generate
                if (leftInclusive && rightInclusive)
                {
                    if (leftProjection <= rightProjection)
                    {
                        generateDownwards(data, leftProjection, rightProjection, sx, sy,
                                          currState.y, true, true);
                    }
                }
                else if (leftProjection < rightProjection)
                {
                    generateDownwards(data, leftProjection, rightProjection, sx, sy,
                                      currState.y, leftInclusive, rightInclusive);
                }
            }
        }
    }


    void LineOfSightScanner::generateUpwards(ScannerStacks& data, Rational leftBound,
                                             Rational rightBound, int sx, int sy,
                                             int currY, bool leftInclusive,
                                             bool rightInclusive) const
    {
        generateAndSplitIntervals(data, currY + 2, currY + 1, sx, sy, leftBound,
                                  rightBound, leftInclusive, rightInclusive);
    }

    void LineOfSightScanner::generateDownwards(ScannerStacks& data, Rational leftBound,
                                               Rational rightBound, int sx, int sy,
                                               int currY, bool leftInclusive,
                                               bool rightInclusive) const
    {
        generateAndSplitIntervals(data, currY - 1, currY - 1, sx, sy, leftBound,
                                  rightBound, leftInclusive, rightInclusive);
    }

    /**
     * Called by generateUpwards / Downwards.
     * Note: Unlike Anya, 0-length intervals are possible.
     */
    void LineOfSightScanner::generateAndSplitIntervals(
        ScannerStacks& data, int checkY, int newY, int sx, int sy, Rational leftBound,
        Rational rightBound, bool leftInclusive, bool rightInclusive) const
    {
        Rational left = leftBound;
        int leftFloor = left.floor();

        // Up: !bottomRightOfBlockedTile && bottomLeftOfBlockedTile
        if (leftInclusive && left.isWholeNumber() &&
            !grid.isBlocked(leftFloor - 1, checkY - 1) &&
            grid.isBlocked(leftFloor, checkY - 1))
        {
            data.stackPush(ScanInterval(newY, left, left, true, true));
        }

        // Divide up the intervals.
        while (true)
        {
            int right = rightDownExtents[checkY * extentsSizeX +
                                         leftFloor];  // it's actually rightDownExtents
                                                      // for exploreDownwards. (thus we
                                                      // use checkY = currY - 2)
            if (rightBound <= right)
                break;  // right < rightBound

            // Only push unblocked ( bottomRightOfBlockedTile )
            if (!grid.isBlocked(right - 1, checkY - 1))
            {
                data.stackPush(
                    ScanInterval(newY, left, Rational(right), leftInclusive, true));
            }

            leftFloor     = right;
            left          = Rational(leftFloor);
            leftInclusive = true;
        }

        // The last interval will always be here.
        // if !bottomLeftOfBlockedTile(leftFloor, checkY)
        if (!grid.isBlocked(leftFloor, checkY - 1))
        {
            data.stackPush(
                ScanInterval(newY, left, rightBound, leftInclusive, rightInclusive));
        }
        else
        {
            // The possibility of there being one degenerate interval at the end. (
            // !bottomLeftOfBlockedTile(xR, checkY) )
            if (rightInclusive && rightBound.isWholeNumber() &&
                !grid.isBlocked(rightBound.n, checkY - 1))
            {
                data.stackPush(ScanInterval(newY, rightBound, rightBound, true, true));
            }
        }
    }

}  // namespace Pathfinding
