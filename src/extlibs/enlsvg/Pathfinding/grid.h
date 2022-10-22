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
        int size_x;
        int size_y;
        int total_size;

        Grid(int size_x, int size_y);
        void printGrid() const;

        // isBlocked, but without boundary checks.
        inline bool isBlockedRaw(int x, int y) const
        {
            return blocked[y * size_x + x];
        }

        // If (x, y) is outside of the grid's boundaries, returns true.
        inline bool isBlocked(int x, int y) const
        {
            return x < 0 || y < 0 || x >= size_x || y >= size_y ||
                   blocked[y * size_x + x];
        }

        inline void setBlocked(int x, int y, bool value)
        {
            blocked[y * size_x + x] = value;
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

        static inline double euclideanDistance(int x_1, int y_1, int x_2, int y_2)
        {
            int dx = x_2 - x_1;
            int dy = y_2 - y_1;
            return sqrt(dx * dx + dy * dy);
        }

        inline bool lineOfSight(int x_1, int y_1, int x_2, int y_2) const
        {
            int dy = y_2 - y_1;
            int dx = x_2 - x_1;

            int f = 0;

            int sign_y   = 1;
            int sign_x   = 1;
            int offset_x = 0;
            int offset_y = 0;

            if (dy < 0)
            {
                dy *= -1;
                sign_y   = -1;
                offset_y = -1;
            }
            if (dx < 0)
            {
                dx *= -1;
                sign_x   = -1;
                offset_x = -1;
            }

            if (dx >= dy)
            {
                while (x_1 != x_2)
                {
                    f += dy;
                    if (f >= dx)
                    {
                        if (isBlocked(x_1 + offset_x, y_1 + offset_y))
                            return false;
                        y_1 += sign_y;
                        f -= dx;
                    }
                    if (f != 0 && isBlocked(x_1 + offset_x, y_1 + offset_y))
                        return false;
                    if (dy == 0 && isBlocked(x_1 + offset_x, y_1) &&
                        isBlocked(x_1 + offset_x, y_1 - 1))
                        return false;

                    x_1 += sign_x;
                }
            }
            else
            {
                while (y_1 != y_2)
                {
                    f += dx;
                    if (f >= dy)
                    {
                        if (isBlocked(x_1 + offset_x, y_1 + offset_y))
                            return false;
                        x_1 += sign_x;
                        f -= dy;
                    }
                    if (f != 0 && isBlocked(x_1 + offset_x, y_1 + offset_y))
                        return false;
                    if (dx == 0 && isBlocked(x_1, y_1 + offset_y) &&
                        isBlocked(x_1 - 1, y_1 + offset_y))
                        return false;

                    y_1 += sign_y;
                }
            }
            return true;
        }

        // Checks whether the path (x_1,y_1),(x_2,y_2),(x_3,y_3) is taut.
        inline bool isTaut(int x_1, int y_1, int x_2, int y_2, int x_3, int y_3) const
        {
            if (x_1 < x_2)
            {
                if (y_1 < y_2)
                {
                    return isTautFromBottomLeft(x_1, y_1, x_2, y_2, x_3, y_3);
                }
                else if (y_2 < y_1)
                {
                    return isTautFromTopLeft(x_1, y_1, x_2, y_2, x_3, y_3);
                }
                else
                {  // y_1 == y_2
                    return isTautFromLeft(x_1, y_1, x_2, y_2, x_3, y_3);
                }
            }
            else if (x_2 < x_1)
            {
                if (y_1 < y_2)
                {
                    return isTautFromBottomRight(x_1, y_1, x_2, y_2, x_3, y_3);
                }
                else if (y_2 < y_1)
                {
                    return isTautFromTopRight(x_1, y_1, x_2, y_2, x_3, y_3);
                }
                else
                {  // y_1 == y_2
                    return isTautFromRight(x_1, y_1, x_2, y_2, x_3, y_3);
                }
            }
            else
            {  // x_2 == x_1
                if (y_1 < y_2)
                {
                    return isTautFromBottom(x_1, y_1, x_2, y_2, x_3, y_3);
                }
                else if (y_2 < y_1)
                {
                    return isTautFromTop(x_1, y_1, x_2, y_2, x_3, y_3);
                }
                else
                {  // y_1 == y_2
                    std::cout << "ERROR: v == u?" << std::endl;
                    return true;
                }
            }
        }

       private:
        inline bool isTautFromBottomLeft(int x_1, int y_1, int x_2, int y_2, int x_3,
                                         int y_3) const
        {
            if (x_3 < x_2 || y_3 < y_2)
                return false;

            int compare_gradients =
                (y_2 - y_1) * (x_3 - x_2) - (y_3 - y_2) * (x_2 - x_1);  // m1 - m2
            if (compare_gradients < 0)
            {  // m1 < m2
                return bottomRightOfBlockedTile(x_2, y_2);
            }
            else if (compare_gradients > 0)
            {  // m1 > m2
                return topLeftOfBlockedTile(x_2, y_2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }


        inline bool isTautFromTopLeft(int x_1, int y_1, int x_2, int y_2, int x_3,
                                      int y_3) const
        {
            if (x_3 < x_2 || y_3 > y_2)
                return false;

            int compare_gradients =
                (y_2 - y_1) * (x_3 - x_2) - (y_3 - y_2) * (x_2 - x_1);  // m1 - m2
            if (compare_gradients < 0)
            {  // m1 < m2
                return bottomLeftOfBlockedTile(x_2, y_2);
            }
            else if (compare_gradients > 0)
            {  // m1 > m2
                return topRightOfBlockedTile(x_2, y_2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }

        inline bool isTautFromBottomRight(int x_1, int y_1, int x_2, int y_2, int x_3,
                                          int y_3) const
        {
            if (x_3 > x_2 || y_3 < y_2)
                return false;

            int compare_gradients =
                (y_2 - y_1) * (x_3 - x_2) - (y_3 - y_2) * (x_2 - x_1);  // m1 - m2
            if (compare_gradients < 0)
            {  // m1 < m2
                return topRightOfBlockedTile(x_2, y_2);
            }
            else if (compare_gradients > 0)
            {  // m1 > m2
                return bottomLeftOfBlockedTile(x_2, y_2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }


        inline bool isTautFromTopRight(int x_1, int y_1, int x_2, int y_2, int x_3,
                                       int y_3) const
        {
            if (x_3 > x_2 || y_3 > y_2)
                return false;

            int compare_gradients =
                (y_2 - y_1) * (x_3 - x_2) - (y_3 - y_2) * (x_2 - x_1);  // m1 - m2
            if (compare_gradients < 0)
            {  // m1 < m2
                return topLeftOfBlockedTile(x_2, y_2);
            }
            else if (compare_gradients > 0)
            {  // m1 > m2
                return bottomRightOfBlockedTile(x_2, y_2);
            }
            else
            {  // m1 == m2
                return true;
            }
        }


        inline bool isTautFromLeft(int x_1, int y_1, int x_2, int y_2, int x_3,
                                   int y_3) const
        {
            if (x_3 < x_2)
                return false;

            int dy = y_3 - y_2;
            if (dy < 0)
            {  // y_3 < y_2
                return topRightOfBlockedTile(x_2, y_2);
            }
            else if (dy > 0)
            {  // y_3 > y_2
                return bottomRightOfBlockedTile(x_2, y_2);
            }
            else
            {  // y_3 == y_2
                return true;
            }
        }

        inline bool isTautFromRight(int x_1, int y_1, int x_2, int y_2, int x_3,
                                    int y_3) const
        {
            if (x_3 > x_2)
                return false;

            int dy = y_3 - y_2;
            if (dy < 0)
            {  // y_3 < y_2
                return topLeftOfBlockedTile(x_2, y_2);
            }
            else if (dy > 0)
            {  // y_3 > y_2
                return bottomLeftOfBlockedTile(x_2, y_2);
            }
            else
            {  // y_3 == y_2
                return true;
            }
        }

        inline bool isTautFromBottom(int x_1, int y_1, int x_2, int y_2, int x_3,
                                     int y_3) const
        {
            if (y_3 < y_2)
                return false;

            int dx = x_3 - x_2;
            if (dx < 0)
            {  // x_3 < x_2
                return topRightOfBlockedTile(x_2, y_2);
            }
            else if (dx > 0)
            {  // x_3 > x_2
                return topLeftOfBlockedTile(x_2, y_2);
            }
            else
            {  // x_3 == x_2
                return true;
            }
        }

        inline bool isTautFromTop(int x_1, int y_1, int x_2, int y_2, int x_3,
                                  int y_3) const
        {
            if (y_3 > y_2)
                return false;

            int dx = x_3 - x_2;
            if (dx < 0)
            {  // x_3 < x_2
                return bottomRightOfBlockedTile(x_2, y_2);
            }
            else if (dx > 0)
            {  // x_3 > x_2
                return bottomLeftOfBlockedTile(x_2, y_2);
            }
            else
            {  // x_3 == x_2
                return true;
            }
        }

       public:
    };

}  // namespace Pathfinding


#endif
