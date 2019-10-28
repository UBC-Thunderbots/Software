#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/polygon/point_concept.hpp>
#include <cmath>
#include <iostream>

#include "software/new_geom/angle.h"
#include "software/new_geom/vector.h"

/**
 * A point in 2D space.
 */
class Point final
{
   public:
    /**
     * Creates a Point at the origin (0, 0).
     */
    explicit Point();

    /**
     * Creates a Point at arbitrary coordinates.
     *
     * @param x the <var>x</var> value of the Point
     * @param y the <var>y</var> value of the Point
     */
    Point(double x, double y);

    /**
     * Creates a new Point that is a copy of the given Point
     *
     * @param the Point to duplicate
     */
    explicit Point(const Point &p);

    /**
     * Creates a new Point from a Vector
     *
     * @param the Vector to create a Point from
     */
    explicit Point(const Vector &v);

    /**
     * Returns the x coordinate of this Point
     *
     * @return the x coordinate of this Point
     */
    double x() const;

    /**
     * Returns the y coordinate of this Point
     *
     * @return the y coordinate of this Point
     */
    double y() const;

    /**
     * Sets the coordinates of this point to the new coordinates
     *
     * @param x the new x coordinate
     * @param y the new y coordinate
     */
    void set(double x, double y);

    /**
     * Sets the x coordinate of this point
     *
     * @param x the new x coordinate
     */
    void setX(double x);

    /**
     * Sets the y coordinate of this point
     *
     * @param y the new y coordinate
     */
    void setY(double y);

    /**
     * Returns the distance between this Point and origin (0,0)
     *
     * @return the distance
     */
    double distanceFromOrigin() const;

    /**
     * Returns the distance between this Point and the given point p
     *
     * @return the distance
     */
    double distanceFromPoint(const Point &p) const;

    /**
     * Returns a new Vector from this Point
     *
     * @return A new vector from this Point
     */
    Vector toVector() const;

    /**
     * Returns a new Point that is this Point rotated counterclockwise by an angle
     * about the origin.
     *
     * @param rot the angle to rotate the Point
     *
     * @return the new Point rotated by rot
     */
    Point rotate(const Angle &rot) const;

    /**
     * Checks whether this Point is close to another Point
     *
     * @param other the other point to check against
     *
     * @param dist the distance to check against
     *
     * @return true if the other point is within the given distance (not inclusive) of
     * this Point
     */
    bool isClose(const Point &other, double dist) const;

    /**
     * Assigns one Point to another
     *
     * @param other the Point whose value should be copied into this Point
     *
     * @return this Point
     */
    Point &operator=(const Point &other);

   private:
    /**
     * The X coordinate of the Point. The variable name starts with an underscore to
     * prevent name conflicts with its accessor function.
     */
    double x_;

    /**
     * The Y coordinate of the Point. The variable name starts with an underscore to
     * prevent name conflicts with its accessor function.
     */
    double y_;
};

/**
 * Adds a vector to a point
 *
 * @param p the Point
 * @param v the Vector
 *
 * @return the Point sum of the given Point and Vector
 */
Point operator+(const Point &p, const Vector &v) __attribute__((warn_unused_result));

/**
 * Adds a vector to a point and sets that point to the resulting sum
 *
 * @param p the Point to add the Vector to
 * @param v the Vector to add
 *
 * @return the new value of Point p
 */
Point &operator+=(Point &p, const Vector &v);

/**
 * Negates a Point
 *
 * @param p the Point to negate
 *
 * @return the point with its coordinates negated
 */
Point operator-(const Point &p) __attribute__((warn_unused_result));

/**
 * Subtracts one Point from another
 *
 * @param p the Point to subtract from
 * @param q the Point to subtract
 *
 * @return the vector-difference of the two points
 */
Vector operator-(const Point &p, const Point &q) __attribute__((warn_unused_result));

/**
 * Prints a point to a stream
 *
 * @param os the stream to print to
 * @param p the Point to print
 *
 * @return the stream with the point printed
 */
std::ostream &operator<<(std::ostream &os, const Point &p);

/**
 * Compares two Points for equality
 *
 * @param p the first Point
 * @param q the second Point
 *
 * @return true if the two points represent the same point, and false otherwise
 */
bool operator==(const Point &p, const Point &q);

/**
 * Compares two points for inequality
 *
 * @param p the first Point
 * @param q the second Point
 *
 * @return true if the two points represent different points, and false otherwise
 */
bool operator!=(const Point &p, const Point &q);

// We need to define a hash function so that the Point class can be used in unordered STL
// containers
// like unordered_set and unordered_map
// https://prateekvjoshi.com/2014/06/05/using-hash-function-in-c-for-user-defined-classes/
namespace std
{
    template <>
    struct hash<Point> final
    {
        size_t operator()(const Point &p) const
        {
            hash<double> h;
            return h(p.x()) * 17 + h(p.y());
        }
    };
}  // namespace std

// Make our Point class "compatible" with boost. This lets us pass our Points directly
// into boost algorithms
BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Point, double, cs::cartesian, x, y, setX, setY)
template <>
struct boost::polygon::geometry_concept<Point>
{
    typedef point_concept type;
};
template <>
struct boost::polygon::point_traits<Point>
{
    typedef int coordinate_type;

    static coordinate_type get(const Point &point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.x() : point.y();
    }
};
