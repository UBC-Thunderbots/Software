#pragma once

class Circle;
class Point;

namespace Navigator
{
    namespace Trespass
    {

        /**
         * Return 0 or 1 to indicate if a point is in a circle
         * 
         * @param[in] Circle circle: the reference ccircle
         * @param[in] Point point: the point of interest
         * 
         * @return 1 if the point is in the circle and 0 otherwise
         */
        unsigned int calcLinearTrespassScore(Circle circle, Point point);
    }
}  // namespace Navigator
