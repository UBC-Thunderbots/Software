#pragma once

#include "software/new_geom/point.h"

class Shot
{
   public:
    /**
     * Creates a shot with the given target and angle
     *
     * @param point The target to shoot at
     * @param angle The angle formed by the shot origin, and the edges of the two
     * obstacles closest to the shot path
     *
     */
    Shot(Point point, Angle angle);

    /**
     * Returns a point, the target to shoot at.
     * @return a point, representing the target to shoot at.
     */
    Point getPointToShootAt() const;

    /**
     * Returns the angle formed by the shot origin, and the edges of the two obstacles
     * closest to the shot path
     * @return the angle formed by the shot origin, and the edges of the two obstacles
     * closest to the shot path
     */
    Angle getOpenAngle() const;

   private:
    /**
     * The target of the shot
     *
     */
    Point point;

    /**
     * The angle formed by the shot origin, and the edges of the two obstacles closest to
     the shot path
     *
                             XXXXO - Obstacle
                         XXXX    |
                     XXXX        |
                 XXXX            |
        Robot - R ============== T - Target
                 XXXX            |
                     XXXX        |
                         XXXX    |
                             XXXXO - Obstacle

        The angle formed by '|' in between the obstacles
        '=' represents the shot vector
     *
     */
    Angle angle;
};
