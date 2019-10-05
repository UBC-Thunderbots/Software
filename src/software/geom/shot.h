#pragma once

#include "software/geom/point.h"

class Shot
{
   public:
    /**
     * Creates a shot with the given target and angle
     *
     * @param point The target to shoot at
     * @param angle The angle between the obstacles on either side of the shot vector
     *
     */
    Shot(Point point, Angle angle);

    /**
     * Returns a point, the target to shoot at.
     * @return a point, representing the target to shoot at.
     */
    Point getPoint() const;

    /**
     * Returns an angle between the obstacles on either side of the shot vector.
     * @return an angle between the obstacles on either side of the shot vector.
     */
    Angle getAngle() const;

   private:
    /**
     * The target of the shot
     *
     */
    Point point;

    /**
     * The angle between the obstacles on either side of the shot vector
     *
     */
    Angle angle;
};
