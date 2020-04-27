#pragma once

#include "software/ai/navigator/obstacle/shape_obstacle.h"
#include "software/new_geom/circle.h"

/**
 * An Obstacle represented by a Circle
 */
class CircleObstacle : public virtual ShapeObstacle
{
   public:
    CircleObstacle() = delete;

    /**
     * Construct a circle obstacle by with a Circle
     *
     * @param circle Circle to make obstacle with
     */
    explicit CircleObstacle(const Circle circle);

    /**
     * Construct a circle obstacle defined by the given parameters
     *
     * @param circle_centre The centre point of the circle
     * @param circle_radius The radius of the circle
     * @param radius_scaling How much to scale the radius
     *
     * @return circle shaped obstacle
     */
    explicit CircleObstacle(const Point& circle_centre, const double circle_radius,
                            const double radius_scaling);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    std::string toString(void) const override;

   private:
    Circle circle_;
};
