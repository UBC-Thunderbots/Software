#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <cmath>
#include <iostream>

#include "geom/angle.h"

/**
 * A point or vector in 2D space.
 * Here, point and vector are used interchangeably. A Point lies on the 2D x-y plane. The
 * corresponding
 * vector can be though of as a vector/line from the origin to the Point on the plane.
 */
class Point final
{
   public:
    // Due to internal representation of doubles being slightly less accurate/consistent
    // with some numbers and operations, we consider points that are very close together
    // to be equal (since they likely are, just possibly slightly misrepresented by the
    // system/compiler). We use this EPSILON as a threshold for comparison. 1e-15 was
    // chosen as a value because doubles have about 16 consistent significant figures.
    // Comparing numbers with 15 significant figures gives us a
    // small buffer while remaining as accurate as possible.
    // http://www.cplusplus.com/forum/beginner/95128/
    static constexpr double EPSILON = 1e-15;

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
    static Point createFromAngle(Angle angle);

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
     * Returns the square of the length of the Point
     *
     * @return the square of the length of the Point
     */
    constexpr double lensq() const;

    /**
     * Returns the length of the Point
     *
     * @return the length of the Point
     */
    double len() const;

    /**
     * Returns the unit vector in the same direction as this Point
     *
     * @return Point a unit vector in the same direction as this Point, or a
     * zero-length Point if this Point is zero
     */
    Point norm() const;

    /**
     * Returns a scaled normalized vector in the same direction as this
     * Point
     *
     * @param length the desired length of the resultant vector
     *
     * @return a vector in the same direction as this Point and with the given length,
     * or a zero-length Point if this Point is zero
     */
    Point norm(double length) const;

    /**
     * Returns the vector perpendicular to this Point
     *
     * @return a vector perpendicular to this Point
     */
    constexpr Point perp() const;

    /**
     * Rotates this Point counterclockwise by an angle
     *
     * @param rot the angle to rotate the vector
     *
     * @return the Point rotated by rot
     */
    Point rotate(Angle rot) const;

    /**
     * Projects this vector onto another vector
     *
     * @param n the vector to project onto
     *
     * @return the component of this Point that is in the same direction as the given
     * Point
     */
    constexpr Point project(const Point &other) const;

    /**
     * Takes the dot product of two vectors
     *
     * @param other the Point to dot against
     *
     * @return the dot product of the points
     */
    constexpr double dot(const Point &other) const;

    /**
     * Takes the cross product of two vectors
     *
     * @param other the Point to cross with
     *
     * @return the z component of the 3-dimensional cross product between this Point and
     * the other point
     */
    constexpr double cross(const Point &other) const;

    /**
     * Returns the direction of this Point
     *
     * @return the direction of this Point, in the range [-π, π], with 0 being
     * the positive x direction, π/2 being up (positive y), etc. like on a standary x-y
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
    constexpr bool isClose(const Point &other) const;

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
    constexpr bool isClose(const Point &other, double dist) const;

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
     * prevent
     * name conflicts with its accessor function.
     */
    double _x;

    /**
     * The Y coordinate of the Point. The variable name starts with an underscore to
     * prevent
     * name conflicts with its accessor function.
     */
    double _y;
};

/**
 * Adds two points
 *
 * @param p the first Point
 * @param q the second Point
 *
 * @return the vector-sum of the two points
 */
constexpr Point operator+(const Point &p, const Point &q);

/**
 * Adds an offset to a Point
 *
 * @param p the Point to add the offset to
 * @param q the offset to add
 *
 * @return the new value of Point p
 */
Point &operator+=(Point &p, const Point &q);

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
constexpr Point operator-(const Point &p, const Point &q)
    __attribute__((warn_unused_result));

/**
 * Subtracts an offset from a Point
 *
 * @paramp the Point to subtract the offset from
 * @param q the offset to subtract
 *
 * @return the new Point with the offest subtracted
 */
Point &operator-=(Point &p, const Point &q);

/**
 * Multiplies a vector by a scalar
 *
 * @param s the scaling factor
 * @param p the vector to scale
 *
 * @return the scaled vector
 */
constexpr Point operator*(double s, const Point &p);

/**
 * Multiplies a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scaling factor
 *
 * @return the scaled vector
 */
constexpr Point operator*(const Point &p, double s);

/**
 * Scales a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scaling factor
 *
 * @return p scaled by the scaling factor
 */
Point &operator*=(Point &p, double s);

/**
 * Divides a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scalar to divide by
 *
 * @return the scaled vector
 */
constexpr Point operator/(const Point &p, double s);

/**
 * Scales a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the sclaing factor
 *
 * @return p scaled by the scaling factor
 */
Point &operator/=(Point &p, double s);

/**
 * Prints a vector to a stream
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
constexpr bool operator==(const Point &p, const Point &q);

/**
 * Compares two vectors for inequality
 *
 * @param p the first Point
 * @param q the second Point
 *
 * @return true if the two points represent different points, and false otherwise
 */
constexpr bool operator!=(const Point &p, const Point &q);

inline Point Point::createFromAngle(Angle angle)
{
    return Point(angle.cos(), angle.sin());
}

inline constexpr Point::Point() : _x(0.0), _y(0.0) {}

inline constexpr Point::Point(double x, double y) : _x(x), _y(y) {}

inline constexpr Point::Point(const Point &p) : _x(p.x()), _y(p.y()) {}

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

inline constexpr double Point::lensq() const
{
    return _x * _x + _y * _y;
}

inline double Point::len() const
{
    return std::hypot(_x, _y);
}

inline Point Point::norm() const
{
    return len() < 1.0e-9 ? Point() : Point(_x / len(), _y / len());
}

inline Point Point::norm(double length) const
{
    return len() < 1.0e-9 ? Point() : Point(_x * length / len(), _y * length / len());
}

inline constexpr Point Point::perp() const
{
    return Point(-_y, _x);
}

inline Point Point::rotate(Angle rot) const
{
    return Point(_x * rot.cos() - _y * rot.sin(), _x * rot.sin() + _y * rot.cos());
}

inline constexpr Point Point::project(const Point &other) const
{
    return dot(other) / other.lensq() * other;
}

inline constexpr double Point::dot(const Point &other) const
{
    return _x * other.x() + _y * other.y();
}

inline constexpr double Point::cross(const Point &other) const
{
    return _x * other.y() - _y * other.x();
}

inline Point &Point::operator=(const Point &q)
{
    _x = q.x();
    _y = q.y();
    return *this;
}

inline Angle Point::orientation() const
{
    return Angle::ofRadians(std::atan2(_y, _x));
}

inline constexpr bool Point::isnan() const
{
    return std::isnan(_x) || std::isnan(_y);
}

inline constexpr bool Point::isClose(const Point &other) const
{
    return Point(_x - other.x(), _y - other.y()).lensq() < 1e-9;
}

inline constexpr bool Point::isClose(const Point &other, double dist) const
{
    return std::pow(_x - other.x(), 2) + std::pow(_y - other.y(), 2) < dist * dist;
}

inline constexpr Point operator+(const Point &p, const Point &q)
{
    return Point(p.x() + q.x(), p.y() + q.y());
}

inline Point &operator+=(Point &p, const Point &q)
{
    p.set(p.x() + q.x(), p.y() + q.y());
    return p;
}

inline constexpr Point operator-(const Point &p)
{
    return Point(-p.x(), -p.y());
}

inline constexpr Point operator-(const Point &p, const Point &q)
{
    return Point(p.x() - q.x(), p.y() - q.y());
}

inline Point &operator-=(Point &p, const Point &q)
{
    p.set(p.x() - q.x(), p.y() - q.y());
    return p;
}

inline constexpr Point operator*(double s, const Point &p)
{
    return Point(p.x() * s, p.y() * s);
}

inline constexpr Point operator*(const Point &p, double s)
{
    return Point(p.x() * s, p.y() * s);
}

inline Point &operator*=(Point &p, double s)
{
    p.set(p.x() * s, p.y() * s);
    return p;
}

inline constexpr Point operator/(const Point &p, double s)
{
    return Point(p.x() / s, p.y() / s);
}

inline Point &operator/=(Point &p, double s)
{
    p.set(p.x() / s, p.y() / s);
    return p;
}

inline std::ostream &operator<<(std::ostream &os, const Point &p)
{
    os << "(" << p.x() << ", " << p.y() << ")";
    return os;
}

inline constexpr bool operator==(const Point &p, const Point &q)
{
    return p.isClose(q, Point::EPSILON);
}

inline constexpr bool operator!=(const Point &p, const Point &q)
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

// Since we also use Points to represent 2D vectors, we also allow
// Points to be referred to as Vectors. This help make interfaces easier to read.
typedef Point Vector;

// Make our Point class "compatible" with boost
BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET(Point, double, cs::cartesian, x, y, setX, setY)
