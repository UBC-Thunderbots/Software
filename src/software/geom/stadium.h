#pragma once

#include "software/geom/convex_shape.h"
#include "software/geom/convex_polygon.h"
#include "software/geom/point.h"
#include "software/geom/segment.h"

/**
 * A tube/stadium/pill/discorectangle shape with a radius and a line segment
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
      * Creates a Stadium with a line segment between arbitrary point and vector originating from said point
      * @param point the center of the first semicircle of the Stadium
      * @param vector the vector from the center of the first semicircle of the Stadium to the second
      * @param radius the radius of the two semicircles of the Stadium
      */
     explicit Stadium(const Point &point, const Vector &vector, double radius);

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
      * Returns the area of this Stadium
      * @return the area of this Stadium
      */
     double area() const override;

     /**
      * Returns the absolute largest x distance between any two points on this Stadium
      *
      * @return the absolute largest x distance between any two points on this Stadium
      */
     double x_span() const;

    /**
     * Returns the absolute largest y distance between any two points on this Stadium
     *
     * @return the absolute largest y distance between any two points on this Stadium
     */
     double y_span() const;

    private:
     Segment length_;
     double radius_;

};

/**
 * Compares two Stadiums for equality
 *
 * @param s1 the first Stadium
 * @param s2 the second Stadium
 *
 * @return true if the two Stadiums represent the same Stadium, and false otherwise
 */
bool operator==(const Stadium &s1, const Stadium &s2);

/**
 * Compares two Stadiums for inequality
 *
 * @param s1 the first Stadium
 * @param s2 the second Stadium
 *
 * @return true if the two Stadiums represent different Stadiums, and false otherwise
 */
bool operator!=(const Stadium &s1, const Stadium &s2);

/**
 * Implements the << operator for printing
 *
 * @param os The stream to print to
 * @param stadium The Stadium to print
 * @return
 */
std::ostream &operator<<(std::ostream &os, const Stadium &stadium);