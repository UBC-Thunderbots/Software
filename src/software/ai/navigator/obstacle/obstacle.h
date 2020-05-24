#pragma once

#include <sstream>
#include <variant>

#include "shared/constants.h"
#include "software/new_geom/circle.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/segment.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersects.h"

/**
 * Represents the underlying shape of the obstacle */
using ObstacleShape = std::variant<Polygon, Circle>;

/**
 * An obstacle is an area to avoid for navigation
 */
class Obstacle
{
   public:
    Obstacle() = delete;

    /**
     * Construct a obstacle with a Circle
     *
     * @param circle Circle to make obstacle with
     */
    explicit Obstacle(const Circle& circle);

    /**
     * Construct a obstacle with a Polygon
     *
     * @param circle Polygon to make obstacle with
     */
    explicit Obstacle(const Polygon& polygon);

    /**
     * Determines whether the given Point is contained within this Obstacle
     *
     * @return whether the Point p is contained within this Obstacle
     */
    bool contains(const Point& p) const;

    /**
     * Gets the minimum distance from the obstacle to the point
     *
     * @param point Point to get distance to
     *
     * @return distance to point
     */
    double distance(const Point& p) const;

    /**
     * Determines whether the given Segment intersects this Obstacle
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

    /**
     * Gets the shape of the obstacle
     *
     * @return The ObstacleShape
     */
    const ObstacleShape getObstacleShape(void) const;

   private:
    ObstacleShape obstacle_shape_;
};

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param obstacle The Obstacle to print
 *
 * @return The output stream with the string representation of the class appended
 */
std::ostream& operator<<(std::ostream& os, const Obstacle& obstacle);
