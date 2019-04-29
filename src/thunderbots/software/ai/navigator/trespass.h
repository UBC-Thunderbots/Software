#pragma once

#include "geom/rectangle.h"

namespace Navigator
{
    namespace Trespass
    {
        /**
         * Returns a trespass score in the range [0, 1] for a rectangle
         * If the point is outside the rectangle, returns 0
         * If the point is at the centre of the rectangle, returns 1
         *
         * @param
         */
        double calcLinearTrespassScore(Rectangle r, Point p);

        /**
         * Returns the binary trespass score of a point and rectangle
         *
         * @param point The point to check for trespassing
         * @param rectangle The rectangle to check for trespassing by the Point parameter
         * @return 1 if the point exists within the rectangle, or on the boundry of the
         * rectangle 0 if the point exists outside of the rectangle
         */
        int calcBinaryTrespassScore(const Rectangle &rectangle, const Point &point);

    }  // namespace Trespass
}  // namespace Navigator
