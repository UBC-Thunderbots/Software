#pragma once

#include "geom/rectangle.h"

namespace Navigator
{
    namespace Trespass
    {
        /**
         * Returns a tresspass score in the range [0, 1] for a rectangle
         * If the point is outside the rectangle, returns 0
         * If the point is at the centre of the rectangle, returns 1
         *
         * @param
         */
        double calcLinearTresspassScore(Rectangle r, Point p);

    }  // namespace Trespass
}  // namespace Navigator
