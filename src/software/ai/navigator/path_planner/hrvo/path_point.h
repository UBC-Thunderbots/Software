#pragma once

#include "software/geom/angle.h"
#include "software/geom/point.h"
/**
 * A path point in an agent's path
 */
class PathPoint
{
public:
    /**
     * @param position The position of this path point
     * @param speed The speed at this path point
     * @param orientation The orientation angle at this path point
     */
    explicit PathPoint(const Point &position, double speed, Angle orientation);

    /**
     * Gets the position of the path point
     * @return Vector position
     */
    Point getPosition() const;

    /**
     * Gets the speed of the path point
     * @return float speed
     */
    double getSpeed() const;

    /**
     * Gets the speed of the path point
     * @return the destination orientation angle
     */
    Angle getOrientation() const;

private:
    // position for the path point
    Point position;
    // desired speed for the path point
    double speed;
    // desired orientation at path point
    Angle orientation;
};
