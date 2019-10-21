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
    explicit constexpr Point();

    /**
     * Creates a unit-magnitude Point from an angle.
     *
     * @param angle the angle
     *
     * @return Point the Point
     */
    static Point createFromAngle(const Angle &angle);

    /**
     * Creates a Point at arbitrary coordinates.
     *
     * @param x the <var>x</var> value of the Point
     * @param y the <var>y</var> value of the Point
     */
    constexpr Point(double x, double y);

    /**
     * Creates a new Point that is a copy of the given Point
     *
     * @param the Point to duplicate
     */
    constexpr Point(const Point &p);

    /**
     * Creates a new Point from a Vector
     *
     * @param the Vector to create a Point from
     */
    constexpr Point(const Vector &v);

    /**
     * Returns the x coordinate of this Point
     *
     * @return the x coordinate of this Point
     */
    constexpr double x() const;

    /**
     * Returns the y coordinate of this Point
     *
     * @return the y coordinate of this Point
     */
    constexpr double y() const;

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
     * Returns the unit vector in direction of this point
     *
     * @return A unit vector in the direction of this Point, or a
     * zero-vector if this Point is zero
     */
    Vector norm() const;

    /**
     * Returns a scaled normalized vector in the same direction as this
     * Point
     *
     * @param length the desired length of the resultant vector
     *
     * @return a vector in the same direction as this Point and with the given length,
     * or a zero-vector if this Point is zero
     */
    Vector norm(double length) const;

    /**
     * Returns a new Point that is this Point rotated counterclockwise by an angle
     *
     * @param rot the angle to rotate the Point
     *
     * @return the new Point rotated by rot
     */
    Point rotate(const Angle &rot) const;

    /**
     * Returns the direction of this Point
     *
     * @return the direction of this Point, in the range [-π, π], with 0 being
     * the positive x direction, π/2 being up (positive y), etc. like on a standard x-y
     * plane
     *
     *              +
     *              | Y
     *              |
     *              |
     *      +---------------+
     *       -X     |      X
     *              |
     *              | -Y
     *              +
     */
    Angle orientation() const;

    /**
     * Checks whether this Point contains NaN in either coordinate
     *
     * @return true if either coordinate is NaN, and false otherwise
     */
    constexpr bool isnan() const;

    /**
     * Checks whether this Point is close to another Point, where “close”
     * is defined as 1.0e-9
     *
     * @param other the other point to check against
     *
     * @return true if the other point is within a distance of 1.0e-9, not inclusive
     */
    bool isClose(const Point &other) const;

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
    double _x;

    /**
     * The Y coordinate of the Point. The variable name starts with an underscore to
     * prevent name conflicts with its accessor function.
     */
    double _y;
};

/**
 * Adds a vector to a point
 *
 * @param p the Point
 * @param v the Vector
 *
 * @return the Point sum of the given Point and Vector
 */
constexpr Point operator+(const Point &p, const Vector &v);

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
constexpr Point operator-(const Point &p) __attribute__((warn_unused_result));

/**
 * Subtracts one Point from another
 *
 * @param p the Point to subtract from
 * @param q the Point to subtract
 *
 * @return the vector-difference of the two points
 */
constexpr Vector operator-(const Point &p, const Point &q)
    __attribute__((warn_unused_result));

/**
 * Prints a point to a stream
 *
 * @param os the stream to print to
 * @param p the Point to print
 *
 * @return the stream with the point printed
 */
inline std::ostream &operator<<(std::ostream &os, const Point &p);

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

inline Point Point::createFromAngle(const Angle &angle)
{
    return Point(angle.cos(), angle.sin());
}

inline constexpr Point::Point() : _x(0.0), _y(0.0) {}

inline constexpr Point::Point(double x, double y) : _x(x), _y(y) {}

inline constexpr Point::Point(const Point &p) : _x(p.x()), _y(p.y()) {}

inline constexpr Point::Point(const Vector &v) : _x(v.x()), _y(v.y()) {}

inline constexpr double Point::x() const
{
    return _x;
}

inline constexpr double Point::y() const
{
    return _y;
}

inline void Point::set(double x, double y)
{
    this->_x = x;
    this->_y = y;
}

inline void Point::setX(double x)
{
    this->_x = x;
}

inline void Point::setY(double y)
{
    this->_y = y;
}

inline double Point::distanceFromOrigin() const
{
    return std::hypot(_x, _y);
}

inline double Point::distanceFromPoint(const Point &p) const
{
    return sqrt(pow((_x - p.x()), 2) + pow((_y - p.y()), 2));
}

inline Vector Point::toVector() const
{
    return Vector(_x, _y);
}

inline Vector Point::norm() const
{
    return distanceFromOrigin() < 1.0e-9
               ? Vector()
               : Vector(_x / distanceFromOrigin(), _y / distanceFromOrigin());
}

inline Vector Point::norm(double length) const
{
    return distanceFromOrigin() < 1.0e-9 ? Vector()
                                         : Vector(_x * length / distanceFromOrigin(),
                                                  _y * length / distanceFromOrigin());
}

inline Point Point::rotate(const Angle &rot) const
{
    return Point(_x * rot.cos() - _y * rot.sin(), _x * rot.sin() + _y * rot.cos());
}

inline Angle Point::orientation() const
{
    return Angle::ofRadians(std::atan2(_y, _x));
}

inline constexpr bool Point::isnan() const
{
    return std::isnan(_x) || std::isnan(_y);
}

inline bool Point::isClose(const Point &other) const
{
    return distanceFromPoint(other) < GeomConstants::NEAR;
}

inline bool Point::isClose(const Point &other, double dist) const
{
    return distanceFromPoint(other) < dist * dist;
}

inline Point &Point::operator=(const Point &q)
{
    _x = q.x();
    _y = q.y();
    return *this;
}

inline constexpr Point operator+(const Point &p, const Vector &v)
{
    return Point(p.x() + v.x(), p.y() + v.y());
}

inline Point &operator+=(Point &p, const Vector &v)
{
    p.set(p.x() + v.x(), p.y() + v.y());
    return p;
}

inline constexpr Point operator-(const Point &p)
{
    return Point(-p.x(), -p.y());
}

inline constexpr Vector operator-(const Point &p, const Point &q)
{
    return Vector(p.x() - q.x(), p.y() - q.y());
}

inline std::ostream &operator<<(std::ostream &os, const Point &p)
{
    os << "(" << p.x() << ", " << p.y() << ")";
    return os;
}

inline bool operator==(const Point &p, const Point &q)
{
    return p.isClose(q, GeomConstants::EPSILON);
}

inline bool operator!=(const Point &p, const Point &q)
{
    return !(p == q);
}

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

    static inline coordinate_type get(const Point &point, orientation_2d orient)
    {
        return (orient == HORIZONTAL) ? point.x() : point.y();
    }
};
