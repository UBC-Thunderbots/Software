#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/polygon/point_concept.hpp>
#include <cmath>
#include <iostream>

#include "software/new_geom/angle.h"

/**
 * A vector in 2D space.
 */
class Vector final
{
public:
    /**
     * Creates a zero-vector.
     */
    explicit constexpr Vector();

    /**
     * Creates a unit-magnitude Vector from an angle.
     *
     * @param angle the angle
     *
     * @return Vector the Vector
     */
    static Vector createFromAngle(Angle angle);

    /**
     * Creates a Vector with arbitrary x and y values.
     *
     * @param x the <var>x</var> value of the Vector
     * @param y the <var>y</var> value of the Vector
     */
    constexpr Vector(double x, double y);

    /**
     * Creates a new Vector that is a copy of the given Vector
     *
     * @param the Vector to duplicate
     */
    constexpr Vector(const Vector &v);

    /**
     * Returns the magnitude in the x-coordinate of this Vector
     *
     * @return the magnitude in the x-coordinate of this Vector
     */
    constexpr double x() const;

    /**
     * Returns the magnitude in the y-coordinate of this Vector
     *
     * @return the magnitude in the y-coordinate of this Vector
     */
    constexpr double y() const;

    /**
     * Sets the magnitudes of this point to the new magnitudes
     *
     * @param x the new magnitude in the x-coordinate
     * @param y the new magnitude in the y-coordinate
     */
    void set(double x, double y);

    /**
     * Sets the magnitude in the x-coordinate of this vector
     *
     * @param x the new x-coordinate magnitude
     */
    void setX(double x);

    /**
     * Sets the magnitude in the y-coordinate of this vector
     *
     * @param y the new y-coordinate magnitude
     */
    void setY(double y);

    /**
     * Returns the square of the length of the Vector
     *
     * @return the square of the length of the Vector
     */
    constexpr double lensq() const;

    /**
     * Returns the length of the Vector
     *
     * @return the length of the Vector
     */
    double len() const;

    /**
     * Returns this Vector as a unit vector in the same direction
     *
     * @return Vector a unit vector in the same direction as this Vector, or a
     * zero-vector if this Vector is a zero-vector
     */
    Vector norm() const;

    /**
     * Returns a scaled normalized vector in the same direction as this
     * Vector
     *
     * @param length the desired length of the resultant vector
     *
     * @return a vector in the same direction as this Vector and with the given length,
     * or a zero-vector if this Vector is a zero-vector
     */
    Vector norm(double length) const;

    /**
     * Returns the vector perpendicular to this Vector
     *
     * @return a vector perpendicular to this Vector
     */
    constexpr Vector perp() const;

    /**
     * Rotates this Vector counterclockwise by an angle
     *
     * @param rot the angle to rotate the vector
     *
     * @return the Vector rotated by rot
     */
    Vector rotate(Angle rot) const;

    /**
     * Projects this vector onto another vector
     *
     * @param the vector to project onto
     *
     * @return the component of this Vector that is in the same direction as the given
     * Vector
     */
    constexpr Vector project(const Vector &other) const;

    /**
     * Takes the dot product of two vectors
     *
     * @param other the Vector to dot against
     *
     * @return the dot product of the vectors
     */
    constexpr double dot(const Vector &other) const;

    /**
     * Takes the cross product of two vectors
     *
     * @param other the Vector to cross with
     *
     * @return the z component of the 3-dimensional cross product between this Vector and
     * the other Vector
     */
    constexpr double cross(const Vector &other) const;

    /**
     * Returns the direction of this Vector
     *
     * @return the direction of this Vector, in the range [-π, π], with 0 being
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
     * Checks whether this Vector contains NaN in either of its x or y magnitude
     *
     * @return true if either x or y magnitude is NaN, and false otherwise
     */
    constexpr bool isnan() const;

    /**
     * Assigns one Vector to another
     *
     * @param other the Vector whose x and y magnitudes should be copied into this Vector
     *
     * @return this Vector
     */
    Vector &operator=(const Vector &other);

private:
    /**
     * The magnitude in the X coordinate of the Vector. The variable name starts with an underscore to
     * prevent name conflicts with its accessor function.
     */
    double _x;

    /**
     * The magnitude in the Y coordinate of the Vector. The variable name starts with an underscore to
     * prevent name conflicts with its accessor function.
     */
    double _y;
};

/**
 * Adds two vectors
 *
 * @param p the first Vector
 * @param q the second Vector
 *
 * @return the vector-sum of the two vectors
 */
constexpr Vector operator+(const Vector &p, const Vector &q);

/**
 * Adds a Vector to another Vector and set the former vector to the sum
 *
 * @param u the Vector to add the other Vector to
 * @param v the other Vector
 *
 * @return the new value of Vector u
 */
Vector &operator+=(Vector &u, const Vector &v);

/**
 * Negates a Vector
 *
 * @param v the Vector to negate
 *
 * @return the vector with its magnitudes negated
 */
constexpr Vector operator-(const Vector &p) __attribute__((warn_unused_result));

/**
 * Subtracts one Vector from another
 *
 * @param u the Vector to subtract from
 * @param v the Vector to subtract
 *
 * @return the vector-difference of the two vectors
 */
constexpr Vector operator-(const Vector &u, const Vector &v)
__attribute__((warn_unused_result));

/**
 * Subtracts from a Vector another Vector and set the former vector to the difference
 *
 * @paramp u the Vector to substract from
 * @param v the Vector to subtract
 *
 * @return the new value of Vector u
 */
Vector &operator-=(Vector &u, const Vector &v);

/**
 * Multiplies a vector by a scalar
 *
 * @param s the scaling factor
 * @param v the vector to scale
 *
 * @return the scaled vector
 */
constexpr Vector operator*(double s, const Vector &v);

/**
 * Multiplies a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scaling factor
 *
 * @return the scaled vector
 */
constexpr Vector operator*(const Vector &p, double s);

/**
 * Scales a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scaling factor
 *
 * @return p scaled by the scaling factor
 */
Vector &operator*=(Vector &p, double s);

/**
 * Divides a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scalar to divide by
 *
 * @return the scaled vector
 */
constexpr Vector operator/(const Vector &p, double s);

/**
 * Scales a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the sclaing factor
 *
 * @return p scaled by the scaling factor
 */
Vector &operator/=(Vector &p, double s);

/**
 * Prints a vector to a stream
 *
 * @param os the stream to print to
 * @param v the Vector to print
 *
 * @return the stream with the Vector printed
 */
inline std::ostream &operator<<(std::ostream &os, const Vector &v);

/**
 * Compares two Vectors for equality
 *
 * @param u the first Vector
 * @param v the second Vector
 *
 * @return true if the two vectors represent the same vector, and false otherwise
 */
constexpr bool operator==(const Vector &u, const Vector &v);

/**
 * Compares two vectors for inequality
 *
 * @param u the first Vector
 * @param v the second Vector
 *
 * @return true if the two vectors represent different vectors, and false otherwise
 */
constexpr bool operator!=(const Vector &u, const Vector &v);

inline Vector Vector::createFromAngle(Angle angle)
{
    return Vector(angle.cos(), angle.sin());
}

inline constexpr Vector::Vector() : _x(0.0), _y(0.0) {}

inline constexpr Vector::Vector(double x, double y) : _x(x), _y(y) {}

inline constexpr Vector::Vector(const Vector &v) : _x(v.x()), _y(v.y()) {}

inline constexpr double Vector::x() const
{
    return _x;
}

inline constexpr double Vector::y() const
{
    return _y;
}

inline void Vector::set(double x, double y)
{
    this->_x = x;
    this->_y = y;
}

inline void Vector::setX(double x)
{
    this->_x = x;
}

inline void Vector::setY(double y)
{
    this->_y = y;
}

inline constexpr double Vector::lensq() const
{
    return _x * _x + _y * _y;
}

inline double Vector::len() const
{
    return std::hypot(_x, _y);
}

inline Vector Vector::norm() const
{
    return len() < 1.0e-9 ? Vector() : Vector(_x / len(), _y / len());
}

inline Vector Vector::norm(double length) const
{
    return len() < 1.0e-9 ? Vector() : Vector(_x * length / len(), _y * length / len());
}

inline constexpr Vector Vector::perp() const
{
    return Vector(-_y, _x);
}

inline Vector Vector::rotate(Angle rot) const
{
    return Vector(_x * rot.cos() - _y * rot.sin(), _x * rot.sin() + _y * rot.cos());
}

inline constexpr Vector Vector::project(const Vector &other) const
{
    return dot(other) / other.lensq() * other;
}

inline constexpr double Vector::dot(const Vector &other) const
{
    return _x * other.x() + _y * other.y();
}

inline constexpr double Vector::cross(const Vector &other) const
{
    return _x * other.y() - _y * other.x();
}

inline Vector &Vector::operator=(const Vector &q)
{
    _x = q.x();
    _y = q.y();
    return *this;
}

inline Angle Vector::orientation() const
{
    return Angle::ofRadians(std::atan2(_y, _x));
}

inline constexpr bool Vector::isnan() const
{
    return std::isnan(_x) || std::isnan(_y);
}

inline constexpr Vector operator+(const Vector &u, const Vector &v)
{
    return Vector(u.x() + v.x(), u.y() + v.y());
}

inline Vector &operator+=(Vector &u, const Vector &v)
{
    u.set(u.x() + v.x(), u.y() + v.y());
    return u;
}

inline constexpr Vector operator-(const Vector &v)
{
    return Vector(-v.x(), -v.y());
}

inline constexpr Vector operator-(const Vector &u, const Vector &v)
{
    return Vector(u.x() - v.x(), u.y() - v.y());
}

inline Vector &operator-=(Vector &u, const Vector &v)
{
    u.set(u.x() - v.x(), u.y() - v.y());
    return u;
}

inline constexpr Vector operator*(double s, const Vector &v)
{
    return Vector(v.x() * s, v.y() * s);
}

inline constexpr Vector operator*(const Vector &v, double s)
{
    return Vector(v.x() * s, v.y() * s);
}

inline Vector &operator*=(Vector &v, double s)
{
    v.set(v.x() * s, v.y() * s);
    return v;
}

inline constexpr Vector operator/(const Vector &v, double s)
{
    return Vector(v.x() / s, v.y() / s);
}

inline Vector &operator/=(Vector &v, double s)
{
    v.set(v.x() / s, v.y() / s);
    return v;
}

inline std::ostream &operator<<(std::ostream &os, const Vector &v)
{
    os << "(" << v.x() << ", " << v.y() << ")";
    return os;
}

inline constexpr bool operator==(const Vector &u, const Vector &v)
{
    return (u-v).lensq() < GeomConstants::EPSILON;
}

inline constexpr bool operator!=(const Vector &u, const Vector &v)
{
    return !(u == v);
}

// We need to define a hash function so that the Vector class can be used in unordered STL
// containers
// like unordered_set and unordered_map
// https://prateekvjoshi.com/2014/06/05/using-hash-function-in-c-for-user-defined-classes/
namespace std
{
    template <>
    struct hash<Vector> final
    {
        size_t operator()(const Vector &v) const
        {
            hash<double> h;
            return h(v.x()) * 17 + h(v.y());
        }
    };
}  // namespace std
