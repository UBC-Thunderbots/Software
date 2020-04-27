#pragma once

#include "software/ai/navigator/obstacle/shape_obstacle.h"
#include "software/new_geom/polygon.h"

/**
 * An Obstacle represented by a Polygon
 */
class PolygonObstacle : public virtual ShapeObstacle
{
   public:
    PolygonObstacle() = delete;

    /**
     * Construct a polygon obstacle by with a Polygon
     * @param polygon Polygon to make obstacle with
     */
    explicit PolygonObstacle(const Polygon polygon);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    std::string toString(void) const override;

   private:
    Polygon polygon_;
};
