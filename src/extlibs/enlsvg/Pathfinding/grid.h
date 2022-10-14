#ifndef _GRID_H_
#define _GRID_H_

#include <cmath>
#include <iostream>
#include <vector>

namespace Pathfinding
{
    class Grid
    {
       private:
        std::vector<bool> blocked;

       public:
        int sizeX;
        int sizeY;
        int totalSize;

        Grid(int sizeX, int sizeY);
        void printGrid() const;

        // isBlocked, but without boundary checks.
        inline bool isBlockedRaw(int x, int y) const
        {
            return blocked[y * sizeX + x];
        }

        // If (x, y) is outside of the grid's boundaries, returns true.
        inline bool isBlocked(int x, int y) const
        {
            return x < 0 || y < 0 || x >= sizeX || y >= sizeY || blocked[y * sizeX + x];
        }

        inline void setBlocked(int x, int y, bool value)
        {
            blocked[y * sizeX + x] = value;
        }

        inline bool isOuterCorner(int x, int y) const
        {
            return (isBlocked(x - 1, y - 1) || isBlocked(x, y)) !=
                   (isBlocked(x, y - 1) || isBlocked(x - 1, y));
        }

        inline bool topRightOfBlockedTile(int x, int y) const
        {
            return isBlocked(x - 1, y - 1);
        }

        inline bool topLeftOfBlockedTile(int x, int y) const
        {
            return isBlocked(x, y - 1);
        }

        inline bool bottomRightOfBlockedTile(int x, int y) const
        {
            return isBlocked(x - 1, y);
        }

        inline bool bottomLeftOfBlockedTile(int x, int y) const
        {
            return isBlocked(x, y);
        }

        static inline double euclideanDistance(int x1, int y1, int x2, int y2)
        {
            int dx = x2 - x1;
            int dy = y2 - y1;
            return sqrt(dx * dx + dy * dy);
        }

        inline bool lineOfSight(int x1, int y1, int x2, int y2) const
        {
            int dy = y2 - y1;
            int dx = x2 - x1;

            int f = 0;

            int signY   = 1;
            int signX   = 1;
            int offsetX = 0;
            int offsetY = 0;

            if (dy < 0)
            {
                dy *= -1;
                signY   = -1;
                offsetY = -1;
            }
            if (dx < 0)
            {
                dx *= -1;
                signX   = -1;
                offsetX = -1;
            }

            if (dx >= dy)
            {
                while (x1 != x2)
                {
                    f += dy;
                    if (f >= dx)
                    {
                        if (isBlocked(x1 + offsetX, y1 + offsetY))
                            return false;
                        y1 += signY;
                        f -= dx;
                    }
                    if (f != 0 && isBlocked(x1 + offsetX, y1 + offsetY))
                        return false;
                    if (dy == 0 && isBlocked(x1 + offsetX, y1) &&
                        isBlocked(x1 + offsetX, y1 - 1))
                        return false;

                    x1 += signX;
                }
            }
            else
            {
                while (y1 != y2)
                {
                    f += dx;
                    if (f >= dy)
                    {
                        if (isBlocked(x1 + offsetX, y1 + offsetY))
                            return false;
                        x1 += signX;
                        f -= dy;
                    }
                    if (f != 0 && isBlocked(x1 + offsetX, y1 + offsetY))
                        return false;
                    if (dx == 0 && isBlocked(x1, y1 + offsetY) &&
                        isBlocked(x1 - 1, y1 + offsetY))
                        return false;

                    y1 += signY;
                }
            }
            return true;
        }

        // Checks whether the path (x1,y1),(x2,y2),(x3,y3) is taut.
        inline bool isTaut(int x1, int y1, int x2, int y2, int x3, int y3) const
        {
            if (x1 < x2)
            {
                if (y1 < y2)
                {
                    return isTautFromBottomLeft(x1, y1, x2, y2, x3, y3);
                }
                else if (y2 < y1)
                {
                    return isTautFromTopLeft(x1, y1, x2, y2, x3, y3);
                }
                else
                {  // y1 == y2
                    return isTautFromLeft(x1, y1, x2, y2, x3, y3);
                }
            }
            else if (x2 < x1)
            {
                if (y1 < y2)
                {
                    return isTautFromBottomRight(x1, y1, x2, y2, x3, y3);
                }
                else if (y2 < y1)
                {
                    return isTautFromTopRight(x1, y1, x2, y2, x3, y3);
                }
                else
                {  // y1 == y2
                    return isTautFromRight(x1, y1, x2, y2, x3, y3);
                }
            }
            else
            {  // x2 == x1
                if (y1 < y2)
                {
                    return isTautFromBottom(x1, y1, x2, y2, x3, y3);
                }
                else if (y2 < y1)
                {
                    return isTautFromTop(x1, y1, x2, y2, x3, y3);
                }
                else
                {  // y1 == y2
                    std::cout << "ERROR: v == u?" << std::endl;
                    return true;
                }
            }
        }

       private:
        inline bool isTautFromBottomLeft(int x1, int y1, int x2, int y2, int x3,
                                         int y3) const
        {
            if (x3 < x2 || y3 < y2)
                return false;

            int compareGradients =
                (y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1);  // m1 - m2
            if (compareGradients < 0)
            {  // m1 < m2
                return bottomRightOfBlockedTile(x2, y2);
            }
            else if (compareGradients > 0)
            {  // m1 > m2
                return topLeftOfBlockedTile(x2, y2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }


        inline bool isTautFromTopLeft(int x1, int y1, int x2, int y2, int x3,
                                      int y3) const
        {
            if (x3 < x2 || y3 > y2)
                return false;

            int compareGradients =
                (y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1);  // m1 - m2
            if (compareGradients < 0)
            {  // m1 < m2
                return bottomLeftOfBlockedTile(x2, y2);
            }
            else if (compareGradients > 0)
            {  // m1 > m2
                return topRightOfBlockedTile(x2, y2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }

        inline bool isTautFromBottomRight(int x1, int y1, int x2, int y2, int x3,
                                          int y3) const
        {
            if (x3 > x2 || y3 < y2)
                return false;

            int compareGradients =
                (y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1);  // m1 - m2
            if (compareGradients < 0)
            {  // m1 < m2
                return topRightOfBlockedTile(x2, y2);
            }
            else if (compareGradients > 0)
            {  // m1 > m2
                return bottomLeftOfBlockedTile(x2, y2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }


        inline bool isTautFromTopRight(int x1, int y1, int x2, int y2, int x3,
                                       int y3) const
        {
            if (x3 > x2 || y3 > y2)
                return false;

            int compareGradients =
                (y2 - y1) * (x3 - x2) - (y3 - y2) * (x2 - x1);  // m1 - m2
            if (compareGradients < 0)
            {  // m1 < m2
                return topLeftOfBlockedTile(x2, y2);
            }
            else if (compareGradients > 0)
            {  // m1 > m2
                return bottomRightOfBlockedTile(x2, y2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }


        inline bool isTautFromLeft(int x1, int y1, int x2, int y2, int x3, int y3) const
        {
            if (x3 < x2)
                return false;

            int dy = y3 - y2;
            if (dy < 0)
            {  // y3 < y2
                return topRightOfBlockedTile(x2, y2);
            }
            else if (dy > 0)
            {  // y3 > y2
                return bottomRightOfBlockedTile(x2, y2);
            }
            else
            {  // y3 == y2
                return true;
            }
        }

        inline bool isTautFromRight(int x1, int y1, int x2, int y2, int x3, int y3) const
        {
            if (x3 > x2)
                return false;

            int dy = y3 - y2;
            if (dy < 0)
            {  // y3 < y2
                return topLeftOfBlockedTile(x2, y2);
            }
            else if (dy > 0)
            {  // y3 > y2
                return bottomLeftOfBlockedTile(x2, y2);
            }
            else
            {  // y3 == y2
                return true;
            }
        }

        inline bool isTautFromBottom(int x1, int y1, int x2, int y2, int x3, int y3) const
        {
            if (y3 < y2)
                return false;

            int dx = x3 - x2;
            if (dx < 0)
            {  // x3 < x2
                return topRightOfBlockedTile(x2, y2);
            }
            else if (dx > 0)
            {  // x3 > x2
                return topLeftOfBlockedTile(x2, y2);
            }
            else
            {  // x3 == x2
                return true;
            }
        }

        inline bool isTautFromTop(int x1, int y1, int x2, int y2, int x3, int y3) const
        {
            if (y3 > y2)
                return false;

            int dx = x3 - x2;
            if (dx < 0)
            {  // x3 < x2
                return bottomRightOfBlockedTile(x2, y2);
            }
            else if (dx > 0)
            {  // x3 > x2
                return bottomLeftOfBlockedTile(x2, y2);
            }
            else
            {  // x3 == x2
                return true;
            }
        }

       public:
    };

}  // namespace Pathfinding


#endif
