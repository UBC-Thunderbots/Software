#pragma once

#include "software/ai/navigator/obstacle/shape_obstacle.h"
#include "software/new_geom/circle.h"

/**
 * An Obstacle represented by a Circle
 */
class CircleObstacle : public ShapeObstacle
{
   public:
    CircleObstacle();

    /**
     * Construct a circle obstacle by with a Circle
     *
     * @param circle Circle to make obstacle with
     */
    explicit CircleObstacle(const Circle circle);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    std::string toString(void) const override;

    /**
     * Gets underlying circle
     *
     * @return circle
     */
    const Circle getCircle(void) const;

   private:
    Circle circle_;
};

std::ostream& operator<<(std::ostream& os, const CircleObstacle& circle_obstacle);
