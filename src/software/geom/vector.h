#pragma once

#include <cmath>
#include <iostream>

#include "software/geom/angle.h"

/**
 * A vector in 2D space.
 */
class Vector final
{
   public:
    /**
     * Creates a zero-vector.
     */
    explicit Vector();

    /**
     * Creates a unit-magnitude Vector from an angle.
     *
     * @param angle the angle
     *
     * @return Vector the Vector
     */
    static Vector createFromAngle(const Angle &angle);

    /**
     * Creates a Vector with arbitrary x and y values.
     *
     * @param x the <var>x</var> value of the Vector
     * @param y the <var>y</var> value of the Vector
     */
    Vector(double x, double y);

    /**
     * Creates a new Vector that is a copy of the given Vector
     *
     * @param the Vector to duplicate
     */
    Vector(const Vector &v);

    /**
     * Returns the magnitude in the x-coordinate of this Vector
     *
     * @return the magnitude in the x-coordinate of this Vector
     */
    double x() const;

    /**
     * Returns the magnitude in the y-coordinate of this Vector
     *
     * @return the magnitude in the y-coordinate of this Vector
     */
    double y() const;

    /**
     * Sets the magnitudes of this vector to the new magnitudes
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
    double lengthSquared() const;

    /**
     * Returns the length of the Vector
     *
     * @return the length of the Vector
     */
    double length() const;

    /**
     * Returns this Vector as a unit vector in the same direction
     *
     * @return Vector a unit vector in the same direction as this Vector, or a
     * zero-vector if this Vector is a zero-vector
     */
    Vector normalize() const;

    /**
     * Returns a scaled normalized vector in the same direction as this
     * Vector
     *
     * @param length the desired length of the resultant vector
     *
     * @return a vector in the same direction as this Vector and with the given length,
     * or a zero-vector if this Vector is a zero-vector
     */
    Vector normalize(double length) const;

    /**
     * Returns the vector perpendicular to this Vector (rotated +90 degrees
     * from the original vector)
     *
     * @return a vector perpendicular to this Vector
     */
    Vector perpendicular() const;

    /**
     * Rotates this Vector counterclockwise by an angle
     *
     * @param rot the angle to rotate the vector
     *
     * @return the Vector rotated by rot
     */
    Vector rotate(const Angle &rot) const;

    /**
     * Projects this vector onto the line formed by another vector
     *
     * @param the vector to project onto
     *
     * @return the projection of this vector onto the line formed by the given Vector
     */
    Vector project(const Vector &other) const;

    /**
     * Takes the dot product of two vectors
     *
     * @param other the Vector to dot against
     *
     * @return the dot product of the vectors
     */
    double dot(const Vector &other) const;

    /**
     * Takes the cross product of two vectors
     *
     * @param other the Vector to cross with
     *
     * @return the z component of the 3-dimensional cross product between this Vector and
     * the other Vector
     */
    double cross(const Vector &other) const;

    /**
     * Calculates the determinant of a 2x2 matrix
     *
     * @param other Vector forming the bottom row of the matrix
     *
     * @return the determinant with rows from the specified vectors
     */
    double determinant(const Vector &other) const;

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
     * Assigns one Vector to another
     *
     * @param other the Vector whose x and y magnitudes should be copied into this Vector
     *
     * @return this Vector
     */
    Vector &operator=(const Vector &other);

    /**
     * Returns true if this vector is to the right of the given vector. Geometrically, in
     * the acute angle given by two vectors, it returns true if this vector is clockwise
     * of the other vector.
     *
     * @param other the Vector to compare this Vector to
     */
    bool isClockwiseOf(const Vector &other) const;

    /**
     * In the acute angle given by the two vectors, this function returns true if this
     * vector is counterclockwise of the other vector.
     *
     * @param other the Vector to compare this Vector to
     */
    bool isCounterClockwiseOf(const Vector &other) const;

   private:
    /**
     * The magnitude in the X coordinate of the Vector. The variable name starts with an
     * underscore to prevent name conflicts with its accessor function.
     */
    double x_;

    /**
     * The magnitude in the Y coordinate of the Vector. The variable name starts with an
     * underscore to prevent name conflicts with its accessor function.
     */
    double y_;
};

/**
 * Adds two vectors
 *
 * @param p the first Vector
 * @param q the second Vector
 *
 * @return the vector-sum of the two vectors
 */
Vector operator+(const Vector &p, const Vector &q) __attribute__((warn_unused_result));

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
Vector operator-(const Vector &p) __attribute__((warn_unused_result));

/**
 * Subtracts one Vector from another
 *
 * @param u the Vector to subtract from
 * @param v the Vector to subtract
 *
 * @return the vector-difference of the two vectors
 */
Vector operator-(const Vector &u, const Vector &v) __attribute__((warn_unused_result));

/**
 * Subtracts from a Vector another Vector and set the former vector to the difference
 *
 * @paramp u the Vector to subtract from
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
Vector operator*(double s, const Vector &v) __attribute__((warn_unused_result));

/**
 * Multiplies a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scaling factor
 *
 * @return the scaled vector
 */
Vector operator*(const Vector &p, double s) __attribute__((warn_unused_result));

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
Vector operator/(const Vector &p, double s) __attribute__((warn_unused_result));

/**
 * Scales a vector by a scalar
 *
 * @param p the vector to scale
 * @param s the scaling factor
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
std::ostream &operator<<(std::ostream &os, const Vector &v);

/**
 * Compares two Vectors for equality
 *
 * @param u the first Vector
 * @param v the second Vector
 *
 * @return true if the two vectors represent the same vector, and false otherwise
 */
bool operator==(const Vector &u, const Vector &v);

/**
 * Compares two vectors for inequality
 *
 * @param u the first Vector
 * @param v the second Vector
 *
 * @return true if the two vectors represent different vectors, and false otherwise
 */
bool operator!=(const Vector &u, const Vector &v);

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
