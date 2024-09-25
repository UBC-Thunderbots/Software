#pragma once

#include "software/geom/point.h"

class Shot
{
   public:
    Shot() = delete;

    /**
     * Creates a shot with the given origin, target, and angle
     *
     * @param origin The origin of the shot
     * @param target The target to shoot at
     * @param angle The angle formed by the shot origin, and the edges of the two
     * obstacles closest to the shot path
     *
     */
    explicit Shot(Point origin, Point target, Angle angle);

    /**
     * Returns the origin of the shot.
     *
     * @return a point representing the shot origin
     */
    const Point &getOrigin() const;

    /**
     * Returns a point, the target to shoot at.
     *
     * @return a point, representing the target to shoot at.
     */
    const Point &getPointToShootAt() const;

    /**
     * Returns the angle formed by the shot origin, and the edges of the two obstacles
     * closest to the shot path
     *
     * @return the angle formed by the shot origin, and the edges of the two obstacles
     * closest to the shot path
     */
    const Angle &getOpenAngle() const;

   private:
    /**
     * The origin of the shot
     */
    Point origin;

    /**
     * The target of the shot
     */
    Point target;

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
