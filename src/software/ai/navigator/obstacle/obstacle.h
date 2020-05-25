#pragma once

#include <memory>
#include <sstream>

#include "shared/constants.h"
#include "software/new_geom/point.h"
#include "software/new_geom/polygon.h"
#include "software/new_geom/segment.h"
#include "software/new_geom/util/distance.h"
#include "software/new_geom/util/intersects.h"

/**
 * An obstacle is an area to avoid for navigation
 */
class Obstacle
{
   public:
    virtual ~Obstacle() = default;

    /**
     * Determines whether the given Point is contained within this Obstacle
     *
     * @return whether the Point p is contained within this Obstacle
     */
    virtual bool contains(const Point& p) const = 0;

    /**
     * Gets the minimum distance from the obstacle to the point
     *
     * @param point Point to get distance to
     *
     * @return distance to point
     */
    virtual double distance(const Point& p) const = 0;

    /**
     * Determines whether the given Segment intersects this Obstacle
     *
     * @return true if the given Segment intersects this Obstacle
     */
    virtual bool intersects(const Segment& segment) const = 0;

    /**
     * Output string to describe the obstacle
     *
     * @return string that describes the obstacle
     */
    virtual std::string toString(void) const = 0;
};

/**
 * We use a pointer to Obstacle to support inheritance
 * Note: this is a convenience typedef
 */
using ObstaclePtr = std::shared_ptr<Obstacle>;

/**
 * Implements the << operator for printing
 *
 * @param ostream The stream to print to
 * @param obstacle_ptr The ObstaclePtr to print
 *
 * @return The output stream with the string representation of the class appended
 */
std::ostream& operator<<(std::ostream& os, const ObstaclePtr& obstacle_ptr);
