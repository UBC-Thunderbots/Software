#pragma once

#include "software/new_geom/point.h"
#include "software/new_geom/segment.h"

/**
 * A ShapeObstacle represents the shape of the obstacle
 */
class ShapeObstacle
{
   public:
    virtual ~ShapeObstacle() = default;

    /**
     * Determines whether the given Point is contained within this ShapeObstacle
     *
     * @return whether the Point p is contained within this ShapeObstacle
     */
    virtual bool contains(const Point& p) const = 0;

    /**
     * Determines whether the given Segment intersects this ShapeObstacle
     *
     * @return true if the given Segment intersects this ShapeObstacle
     */
    virtual bool intersects(const Segment& segment) const = 0;

    /**
     * Output string to describe the obstacle
     *
     * @return string that describes the obstacle
     */
    virtual std::string toString(void) const;
};
