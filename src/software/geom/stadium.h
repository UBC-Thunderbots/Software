#pragma once

#include "software/geom/convex_shape.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"
#include "software/geom/rectangle.h"

/**
 * A stadium/pill/discorectangle shape with a radius and a line segment
 */
class Stadium : public ConvexShape {
    public:
     /**
      * Creates a Stadium with radius 0 and line segment from (0,0) to (0,0)
      */
     explicit Stadium();

     /**
      * Creates a Stadium with arbitrary line segment and radius
      * @param length the line segment between the centers of the two semicircles of the Stadium
      * @param radius the radius of the two semicircles of the Stadium
      */
     explicit Stadium(const Segment &length, double radius);

     /**
      * Creates a Stadium with line segment between two arbitrary points and radius
      * @param point1 the center of the first semicircle of the Stadium
      * @param point2 the center of the second semicircle of the Stadium
      * @param radius the radius of the two semicircles of the Stadium
      */
     explicit Stadium(const Point &point1, const Point &point2, double radius);

     /**
      * Returns the line Segment defining the top and bottom length of this Stadium
      * @return the line segment between the centers of the semicircles of this Stadium
      */
     Segment length() const;

     /**
      * Returns the radius of the semicircles of this Stadium
      * @return the radius of the semicircles of this Stadium
      */
     double radius() const;

    /**
     * Returns the inner rectangle of this Stadium
     * @return the inner rectangle of this Stadium
     */
    //double rectangle() const;

    /**
     * Returns the area of this Stadium
     * @return the area of this Stadium
     */
     double area() const override;

    private:
     Segment length_;
     double radius_;
};
