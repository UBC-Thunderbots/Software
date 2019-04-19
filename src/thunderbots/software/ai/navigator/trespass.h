#pragma once

class Circle;
class Point;

namespace Navigator::Trespass
{
    /**
     * Return 0 or 1 to indicate if a point is in a circle
     *
     * @param[in] Circle circle: the reference ccircle
     * @param[in] Point point: the point of interest
     *
     * @return 1 if the point is in the circle and 0 otherwise
     */
    unsigned int calcBinaryTrespassScore(Circle circle, Point point);

    /**
     * Calculates the linear score of a point with
     * respect to a circle
     *
     * @param[in] Circle circle: the reference ccircle
     * @param[in] Point point: the point of interest
     *
     * @return 1 if the point is at the circle origin,
     *         0 if the point is outside the circle, and
     *         between 0-1 if the point is somewhere in between
     */
    double calcLinearTrespassScore(Circle circle, Point point);

}  // namespace Navigator::Trespass
