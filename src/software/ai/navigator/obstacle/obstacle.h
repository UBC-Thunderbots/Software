#pragma once

#include <sstream>

#include "shared/constants.h"
#include "software/ai/navigator/obstacle/shape_obstacle.h"
#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/segment.h"

/**
 * An obstacle is an area to avoid for navigation
 */
class Obstacle
{
   public:
    Obstacle() = delete;

    /**
     * Create Obstacle from a ShapeObstacle pointer
     */
    explicit Obstacle(const std::shared_ptr<ShapeObstacle> shape_obstacle_ptr);

    /**
     * Create Obstacle from a Circle
     */
    explicit Obstacle(const Circle& circle);

    /**
     * Create Obstacle from a Polygon
     */
    explicit Obstacle(const Polygon& polygon);

    /**
     * Determines whether the given Point is contained within this Obstacle
     *
     * @param point Point to check contain for
     *
     * @return whether the Point p is contained within this Obstacle
     */
    bool contains(const Point& point) const;

    /**
     * Gets the minimum distance from the obstacle to the point
     *
     * @param point Point to get distance to
     *
     * @return distance to point
     */
    double distance(const Point& point) const;

    /**
     * Determines whether the given Segment intersects this Obstacle
     *
     * @param segment Segment to check intersects for
     *
     * @return true if the given Segment intersects this Obstacle
     */
    bool intersects(const Segment& segment) const;

    /**
     * Output string to describe the obstacle
     *
     * @return string that describes the obstacle
     */
    std::string toString(void) const;

   private:
    std::shared_ptr<ShapeObstacle> shape_obstacle_ptr_;
};

std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle);
