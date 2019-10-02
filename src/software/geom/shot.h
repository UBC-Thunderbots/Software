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
