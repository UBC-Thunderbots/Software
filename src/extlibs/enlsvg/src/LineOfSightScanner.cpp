#include "extlibs/enlsvg/Pathfinding/LineOfSightScanner.h"

#include "extlibs/enlsvg/Pathfinding/Grid.h"

namespace Pathfinding
{
    LineOfSightScanner::LineOfSightScanner(const Grid& grid)
        : grid(grid),
          size_x(grid.size_x),
          size_y(grid.size_y),
          extents_size_x(grid.size_x + 1)
    {
        computeExtents();
    }

    void LineOfSightScanner::computeAllDirNeighbours(ScannerStacks& data, int s_x,
                                                     int s_y) const
    {
        data.clear();

        generateAllDirectionStartingStates(data, s_x, s_y);
        exploreStates(data, s_x, s_y);
    }

    void LineOfSightScanner::computeTautDirNeighbours(ScannerStacks& data, int s_x,
                                                      int s_y) const
    {
        data.clear();

        generateTautDirectionStartingStates(data, s_x, s_y);
        exploreStates(data, s_x, s_y);
    }


    void LineOfSightScanner::computeExtents()
    {
        right_down_extents.resize(extents_size_x * (size_y + 2));
        left_down_extents.resize(extents_size_x * (size_y + 2));

        for (int y = 0; y < size_y + 2; ++y)
        {
            bool last_is_blocked = true;
            int last_x           = -1;
            for (int x = 0; x <= size_x; ++x)
            {
                left_down_extents[y * extents_size_x + x] = last_x;
                if (grid.isBlocked(x, y - 1) != last_is_blocked)
                {
                    last_x          = x;
                    last_is_blocked = !last_is_blocked;
                }
            }
            last_is_blocked = true;
            last_x          = size_x + 1;
            for (int x = size_x; x >= 0; --x)
            {
                right_down_extents[y * extents_size_x + x] = last_x;
                if (grid.isBlocked(x - 1, y - 1) != last_is_blocked)
                {
                    last_x          = x;
                    last_is_blocked = !last_is_blocked;
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
                                                                 int s_x, int s_y) const
    {
        bool bottom_left_of_blocked  = grid.bottomLeftOfBlockedTile(s_x, s_y);
        bool bottom_right_of_blocked = grid.bottomRightOfBlockedTile(s_x, s_y);
        bool top_left_of_blocked     = grid.topLeftOfBlockedTile(s_x, s_y);
        bool top_right_of_blocked    = grid.topRightOfBlockedTile(s_x, s_y);

        // Generate up-left direction
        if (top_right_of_blocked || bottom_left_of_blocked)
        {
            Fraction left_extent(leftUpExtent(s_x, s_y));
            Fraction right_extent(s_x);

            generateUpwards(data, left_extent, right_extent, s_x, s_y, s_y, true, true);
        }

        // Generate up-right direction
        if (bottom_right_of_blocked || top_left_of_blocked)
        {
            Fraction left_extent(s_x);
            Fraction right_extent(rightUpExtent(s_x, s_y));

            generateUpwards(data, left_extent, right_extent, s_x, s_y, s_y, true, true);
        }

        // Generate down-left direction
        if (bottom_right_of_blocked || top_left_of_blocked)
        {
            Fraction left_extent(leftDownExtent(s_x, s_y));
            Fraction right_extent(s_x);

            generateDownwards(data, left_extent, right_extent, s_x, s_y, s_y, true, true);
        }

        // Generate down-right direction
        if (top_right_of_blocked || bottom_left_of_blocked)
        {
            Fraction left_extent(s_x);
            Fraction right_extent(rightDownExtent(s_x, s_y));

            generateDownwards(data, left_extent, right_extent, s_x, s_y, s_y, true, true);
        }

        // Search leftwards
        if (!top_right_of_blocked || !bottom_right_of_blocked)
        {
            int x = leftAnyExtent(s_x, s_y);
            int y = s_y;
            if (!(grid.topRightOfBlockedTile(x, y) &&
                  grid.bottomRightOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }

        // Search rightwards
        if (!top_left_of_blocked || !bottom_left_of_blocked)
        {
            int x = rightAnyExtent(s_x, s_y);
            int y = s_y;
            if (!(grid.topLeftOfBlockedTile(x, y) && grid.bottomLeftOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }
    }

    void LineOfSightScanner::generateAllDirectionStartingStates(ScannerStacks& data,
                                                                int s_x, int s_y) const
    {
        bool bottom_left_of_blocked  = grid.bottomLeftOfBlockedTile(s_x, s_y);
        bool bottom_right_of_blocked = grid.bottomRightOfBlockedTile(s_x, s_y);
        bool top_left_of_blocked     = grid.topLeftOfBlockedTile(s_x, s_y);
        bool top_right_of_blocked    = grid.topRightOfBlockedTile(s_x, s_y);

        // Generate up
        if (!bottom_left_of_blocked || !bottom_right_of_blocked)
        {
            Fraction left_extent, right_extent;

            if (bottom_left_of_blocked)
            {
                // Explore up-left
                left_extent  = Fraction(leftUpExtent(s_x, s_y));
                right_extent = Fraction(s_x);
            }
            else if (bottom_right_of_blocked)
            {
                // Explore up-right
                left_extent  = Fraction(s_x);
                right_extent = Fraction(rightUpExtent(s_x, s_y));
            }
            else
            {
                // Explore up-left-right
                left_extent  = Fraction(leftUpExtent(s_x, s_y));
                right_extent = Fraction(rightUpExtent(s_x, s_y));
            }

            generateUpwards(data, left_extent, right_extent, s_x, s_y, s_y, true, true);
        }

        // Generate down
        if (!top_left_of_blocked || !top_right_of_blocked)
        {
            Fraction left_extent, right_extent;

            if (top_left_of_blocked)
            {
                // Explore down-left
                left_extent  = Fraction(leftDownExtent(s_x, s_y));
                right_extent = Fraction(s_x);
            }
            else if (top_right_of_blocked)
            {
                // Explore down-right
                left_extent  = Fraction(s_x);
                right_extent = Fraction(rightDownExtent(s_x, s_y));
            }
            else
            {
                // Explore down-left-right
                left_extent  = Fraction(leftDownExtent(s_x, s_y));
                right_extent = Fraction(rightDownExtent(s_x, s_y));
            }

            generateDownwards(data, left_extent, right_extent, s_x, s_y, s_y, true, true);
        }

        // Search leftwards
        if (!top_right_of_blocked || !bottom_right_of_blocked)
        {
            int x = leftAnyExtent(s_x, s_y);
            int y = s_y;
            if (!(grid.topRightOfBlockedTile(x, y) &&
                  grid.bottomRightOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }

        // Search rightwards
        if (!top_left_of_blocked || !bottom_left_of_blocked)
        {
            int x = rightAnyExtent(s_x, s_y);
            int y = s_y;
            if (!(grid.topLeftOfBlockedTile(x, y) && grid.bottomLeftOfBlockedTile(x, y)))
            {
                data.addSuccessor(GridVertex(x, y));
            }
        }
    }

    void LineOfSightScanner::exploreStates(ScannerStacks& data, int s_x, int s_y) const
    {
        while (data.interval_stack.size() > 0)
        {
            ScanInterval curr_state = data.stackPop();
            bool left_inclusive     = curr_state.left_inclusive;
            bool right_inclusive    = curr_state.right_inclusive;

            bool zero_length_interval = (curr_state.x_right == curr_state.x_left);
            if (curr_state.y > s_y)
            {
                // Upwards

                // Insert endpoints if integer.
                if (left_inclusive && curr_state.x_left.isWholeNumber())
                {
                    /* The two cases   _
                     *  _             |X|
                     * |X|'.           ,'
                     *      '.       ,'
                     *        B     B
                     */

                    int x                          = curr_state.x_left.n;
                    int y                          = curr_state.y;
                    bool top_right_of_blocked_tile = grid.topRightOfBlockedTile(x, y);
                    bool bottom_right_of_blocked_tile =
                        grid.bottomRightOfBlockedTile(x, y);

                    if (x <= s_x && top_right_of_blocked_tile &&
                        !bottom_right_of_blocked_tile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        left_inclusive = false;
                    }
                    else if (s_x <= x && bottom_right_of_blocked_tile &&
                             !top_right_of_blocked_tile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        left_inclusive = false;
                    }
                }
                if (right_inclusive && curr_state.x_right.isWholeNumber())
                {
                    /*   _   The two cases
                     *  |X|             _
                     *  '.           ,'|X|
                     *    '.       ,'
                     *      B     B
                     */

                    int x                            = curr_state.x_right.n;
                    int y                            = curr_state.y;
                    bool bottom_left_of_blocked_tile = grid.bottomLeftOfBlockedTile(x, y);
                    bool top_left_of_blocked_tile    = grid.topLeftOfBlockedTile(x, y);

                    if (x <= s_x && bottom_left_of_blocked_tile &&
                        !top_left_of_blocked_tile)
                    {
                        if (left_inclusive || !zero_length_interval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            right_inclusive = false;
                        }
                    }
                    else if (s_x <= x && top_left_of_blocked_tile &&
                             !bottom_left_of_blocked_tile)
                    {
                        if (left_inclusive || !zero_length_interval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            right_inclusive = false;
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
                int dy = curr_state.y - s_y;
                Fraction left_projection =
                    (curr_state.x_left - s_x).multiplyDivide(dy + 1, dy) + s_x;

                int left_bound = leftUpExtent(curr_state.x_left.ceil(), curr_state.y);
                if (curr_state.x_left.isWholeNumber() &&
                    grid.bottomRightOfBlockedTile(curr_state.x_left.n, curr_state.y))
                    left_bound = curr_state.x_left.n;

                if (left_projection < left_bound)
                {  // left_projection < left_bound
                    left_projection = Fraction(left_bound);
                    left_inclusive  = true;
                }

                // (Px-Bx)*(Py-By+1)/(Py-By) + Bx
                Fraction right_projection =
                    (curr_state.x_right - s_x).multiplyDivide(dy + 1, dy) + s_x;

                int right_bound = rightUpExtent(curr_state.x_right.floor(), curr_state.y);
                if (curr_state.x_right.isWholeNumber() &&
                    grid.bottomLeftOfBlockedTile(curr_state.x_right.n, curr_state.y))
                    right_bound = curr_state.x_right.n;

                if (right_projection > right_bound)
                {  // right_bound < right_projection
                    right_projection = Fraction(right_bound);
                    right_inclusive  = true;
                }

                // Call Generate
                if (left_inclusive && right_inclusive)
                {
                    if (left_projection <= right_projection)
                    {
                        generateUpwards(data, left_projection, right_projection, s_x, s_y,
                                        curr_state.y, true, true);
                    }
                }
                else if (left_projection < right_projection)
                {
                    generateUpwards(data, left_projection, right_projection, s_x, s_y,
                                    curr_state.y, left_inclusive, right_inclusive);
                }
            }
            else
            {
                // Upwards

                // Insert endpoints if integer.
                if (left_inclusive && curr_state.x_left.isWholeNumber())
                {
                    /* The two cases
                     *        B     B
                     *  _   ,'       '.
                     * |X|.'           '.
                     *                |X|
                     */

                    int x = curr_state.x_left.n;
                    int y = curr_state.y;
                    bool bottom_right_of_blocked_tile =
                        grid.bottomRightOfBlockedTile(x, y);
                    bool top_right_of_blocked_tile = grid.topRightOfBlockedTile(x, y);

                    if (x <= s_x && bottom_right_of_blocked_tile &&
                        !top_right_of_blocked_tile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        left_inclusive = false;
                    }
                    else if (s_x <= x && top_right_of_blocked_tile &&
                             !bottom_right_of_blocked_tile)
                    {
                        data.addSuccessor(GridVertex(x, y));
                        left_inclusive = false;
                    }
                }
                if (right_inclusive && curr_state.x_right.isWholeNumber())
                {
                    /*       The two cases
                     *      B     B
                     *    .'       '.   _
                     *  .'           '.|X|
                     *  |X|
                     */

                    int x                            = curr_state.x_right.n;
                    int y                            = curr_state.y;
                    bool top_left_of_blocked_tile    = grid.topLeftOfBlockedTile(x, y);
                    bool bottom_left_of_blocked_tile = grid.bottomLeftOfBlockedTile(x, y);

                    if (x <= s_x && top_left_of_blocked_tile &&
                        !bottom_left_of_blocked_tile)
                    {
                        if (left_inclusive || !zero_length_interval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            right_inclusive = false;
                        }
                    }
                    else if (s_x <= x && bottom_left_of_blocked_tile &&
                             !top_left_of_blocked_tile)
                    {
                        if (left_inclusive || !zero_length_interval)
                        {
                            data.addSuccessor(GridVertex(x, y));
                            right_inclusive = false;
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
                int dy = s_y - curr_state.y;
                Fraction left_projection =
                    (curr_state.x_left - s_x).multiplyDivide(dy + 1, dy) + s_x;

                int left_bound = leftDownExtent(curr_state.x_left.ceil(), curr_state.y);
                if (curr_state.x_left.isWholeNumber() &&
                    grid.topRightOfBlockedTile(curr_state.x_left.n, curr_state.y))
                    left_bound = curr_state.x_left.n;

                if (left_projection < left_bound)
                {  // left_projection < left_bound
                    left_projection = Fraction(left_bound);
                    left_inclusive  = true;
                }

                // (Px-Bx)*(Py-By+1)/(Py-By) + Bx
                Fraction right_projection =
                    (curr_state.x_right - s_x).multiplyDivide(dy + 1, dy) + s_x;

                int right_bound =
                    rightDownExtent(curr_state.x_right.floor(), curr_state.y);
                if (curr_state.x_right.isWholeNumber() &&
                    grid.topLeftOfBlockedTile(curr_state.x_right.n, curr_state.y))
                    right_bound = curr_state.x_right.n;

                if (right_projection > right_bound)
                {  // right_bound < right_projection
                    right_projection = Fraction(right_bound);
                    right_inclusive  = true;
                }

                // Call Generate
                if (left_inclusive && right_inclusive)
                {
                    if (left_projection <= right_projection)
                    {
                        generateDownwards(data, left_projection, right_projection, s_x,
                                          s_y, curr_state.y, true, true);
                    }
                }
                else if (left_projection < right_projection)
                {
                    generateDownwards(data, left_projection, right_projection, s_x, s_y,
                                      curr_state.y, left_inclusive, right_inclusive);
                }
            }
        }
    }


    void LineOfSightScanner::generateUpwards(ScannerStacks& data, Rational left_bound,
                                             Rational right_bound, int s_x, int s_y,
                                             int curr_y, bool left_inclusive,
                                             bool right_inclusive) const
    {
        generateAndSplitIntervals(data, curr_y + 2, curr_y + 1, s_x, s_y, left_bound,
                                  right_bound, left_inclusive, right_inclusive);
    }

    void LineOfSightScanner::generateDownwards(ScannerStacks& data, Rational left_bound,
                                               Rational right_bound, int s_x, int s_y,
                                               int curr_y, bool left_inclusive,
                                               bool right_inclusive) const
    {
        generateAndSplitIntervals(data, curr_y - 1, curr_y - 1, s_x, s_y, left_bound,
                                  right_bound, left_inclusive, right_inclusive);
    }

    /**
     * Called by generateUpwards / Downwards.
     * Note: Unlike Anya, 0-length intervals are possible.
     */
    void LineOfSightScanner::generateAndSplitIntervals(ScannerStacks& data, int check_y,
                                                       int new_y, int s_x, int s_y,
                                                       Rational left_bound,
                                                       Rational right_bound,
                                                       bool left_inclusive,
                                                       bool right_inclusive) const
    {
        Rational left  = left_bound;
        int left_floor = left.floor();

        // Up: !bottomRightOfBlockedTile && bottomLeftOfBlockedTile
        if (left_inclusive && left.isWholeNumber() &&
            !grid.isBlocked(left_floor - 1, check_y - 1) &&
            grid.isBlocked(left_floor, check_y - 1))
        {
            data.stackPush(ScanInterval(new_y, left, left, true, true));
        }

        // Divide up the intervals.
        while (true)
        {
            int right =
                right_down_extents[check_y * extents_size_x +
                                   left_floor];  // it's actually right_down_extents
                                                 // for exploreDownwards. (thus we
                                                 // use check_y = currY - 2)
            if (right_bound <= right)
                break;  // right < right_bound

            // Only push unblocked ( bottomRightOfBlockedTile )
            if (!grid.isBlocked(right - 1, check_y - 1))
            {
                data.stackPush(
                    ScanInterval(new_y, left, Rational(right), left_inclusive, true));
            }

            left_floor     = right;
            left           = Rational(left_floor);
            left_inclusive = true;
        }

        // The last interval will always be here.
        // if !bottomLeftOfBlockedTile(left_floor, check_y)
        if (!grid.isBlocked(left_floor, check_y - 1))
        {
            data.stackPush(
                ScanInterval(new_y, left, right_bound, left_inclusive, right_inclusive));
        }
        else
        {
            // The possibility of there being one degenerate interval at the end. (
            // !bottomLeftOfBlockedTile(x_right, check_y) )
            if (right_inclusive && right_bound.isWholeNumber() &&
                !grid.isBlocked(right_bound.n, check_y - 1))
            {
                data.stackPush(ScanInterval(new_y, right_bound, right_bound, true, true));
            }
        }
    }

}  // namespace Pathfinding
