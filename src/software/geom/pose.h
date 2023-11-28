#pragma once

#include "software/geom/angle.h"
#include "software/geom/point.h"

/**
 * A pose represents a position in 2D space: x, y and yaw
 */
class Pose final
{
    public:
        /** Default constructor for (0, 0) and 0 radian **/
        explicit Pose();

        /**
         * Constructor
         *
         * @param point         2D x, y location
         * @param orientation   orientation in 2D space
         */
        Pose(const Point& point, const Angle& orientation);

        /**
         * Constructor
         *
         * @param x             2D x location
         * @param y             2D y location
         * @param oreintation   orientation in 2D space
         */
        Pose(double x, double y, const Angle& orientation);

        /**
         * Return 2D position of this location
         *
         * @returns 2D (x, y) coordinate
         */
        Point point() const;

        /**
         * Return 2D orientation of this location
         *
         * @returns orientation of this location
         */
        Angle orientation() const;

    private:
        Point point_;
        Angle orientation_;
};
