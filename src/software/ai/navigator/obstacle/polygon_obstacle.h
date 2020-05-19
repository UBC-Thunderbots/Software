#pragma once

#include "software/ai/navigator/obstacle/obstacle.h"
#include "software/new_geom/polygon.h"

/**
 * An Obstacle represented by a Polygon
 */
class PolygonObstacle : public Obstacle
{
   public:
    PolygonObstacle() = delete;

    /**
     * Construct a polygon obstacle with a Polygon
     * @param polygon Polygon to make obstacle with
     */
    explicit PolygonObstacle(const Polygon& polygon);

    bool contains(const Point& p) const override;
    double distance(const Point& p) const override;
    bool intersects(const Segment& segment) const override;
    std::string toString(void) const override;

    /**
     * Gets the underlying polygon
     *
     * @return polygon
     */
    const Polygon getPolygon(void) const;

   private:
    Polygon polygon_;
};

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param polygon_obstacle The PolygonObstacle to print
 *
 * @return The output stream with the string representation of the class appended
 */
std::ostream& operator<<(std::ostream& os, const PolygonObstacle& polygon_obstacle);
