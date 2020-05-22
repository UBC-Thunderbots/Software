#pragma once

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/new_geom/convex_polygon.h"

/**
 * An Obstacle represented by a ConvexPolygon
 */
class ConvexPolygonObstacle : public Obstacle
{
   public:
    ConvexPolygonObstacle() = delete;

    /**
     * Construct a convex_polygon obstacle with a Polygon
     * @param convex_polygon ConvexPolygon to make obstacle with
     */
    explicit ConvexPolygonObstacle(const ConvexPolygon& convex_polygon);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    std::string toString(void) const override;
    const ObstacleShape getObstacleShape(void) const override;

   private:
    ConvexPolygon convex_polygon_;
};

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param convex_polygon_obstacle The ConvexPolygonObstacle to print
 *
 * @return The output stream with the string representation of the class appended
 */
std::ostream& operator<<(std::ostream& os,
                         const ConvexPolygonObstacle& convex_polygon_obstacle);
