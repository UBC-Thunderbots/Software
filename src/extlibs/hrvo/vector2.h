#pragma once

#include <cmath>
#include <iosfwd>

/**
 * A vector in two dimensions.
 */
class Vector2
{
   public:
    Vector2() : x_(0.0f), y_(0.0f) {}

    /**
     * @param x  The x-coordinate of the vector.
     * @param y  The y-coordinate of the vector.
     */
    Vector2(float x, float y) : x_(x), y_(y) {}

    /**
     * Returns the x-coordinate of this vector.
     *
     * @return The x-coordinate of the vector.
     */
    float getX() const
    {
        return x_;
    }

    /**
     * Returns the y-coordinate of this vector.
     *
     * @return  The y-coordinate of the vector.
     */
    float getY() const
    {
        return y_;
    }

    /**
     * Sets the x-coordinate of this vector.
     *
     * @param x  The replacement x-coordinate.
     */
    void setX(float x)
    {
        x_ = x;
    }

    /**
     * Sets the y-coordinate of this vector.
     *
     * @param y  The replacement y-coordinate.
     */
    void setY(float y)
    {
        y_ = y;
    }

    /**
     * Computes the negation of this vector.
     *
     * @return  The negation of this vector.
     */
    Vector2 operator-() const
    {
        return Vector2(-x_, -y_);
    }

    /**
     * Computes the dot product of this vector with the specified vector.
     *
     * @param other  The vector with which the dot product should be computed.
     * @return  The dot product of this vector with a specified vector.
     */
    float operator*(const Vector2 &other) const
    {
        return x_ * other.x_ + y_ * other.y_;
    }

    /**
     * Computes the scalar multiplication of this vector with the specified scalar
     * value.
     *
     * @param scalar  The scalar value with which the scalar multiplication should be
     * computed.
     * @return  The scalar multiplication of this vector with a specified scalar value.
     */
    Vector2 operator*(float scalar) const
    {
        return Vector2(x_ * scalar, y_ * scalar);
    }

    /**
     * Computes the scalar division of this vector with the specified scalar value.
     *
     * @param scalar  The scalar value with which the scalar division should be computed.
     * @return  The scalar division of this vector with a specified scalar value.
     */
    Vector2 operator/(float scalar) const
    {
        const float invScalar = 1.0f / scalar;

        return Vector2(x_ * invScalar, y_ * invScalar);
    }

    /**
     * Computes the vector sum of this vector with the specified vector.
     *
     * @param other  The vector with which the vector sum should be computed.
     * @return  The vector sum of this vector with a specified vector.
     */
    Vector2 operator+(const Vector2 &other) const
    {
        return Vector2(x_ + other.x_, y_ + other.y_);
    }

    /**
     * Computes the vector difference of this vector with the specified vector.
     *
     * @param other  The vector with which the vector difference should be computed.
     * @return  The vector difference of this vector with a specified vector.
     */
    Vector2 operator-(const Vector2 &other) const
    {
        return Vector2(x_ - other.x_, y_ - other.y_);
    }

    /**
     * Tests this vector for equality with the specified vector.
     *
     * @param other  The vector with which to test for equality.
     * @return  True if the vectors are equal.
     */
    bool operator==(const Vector2 &other) const
    {
        return x_ == other.x_ && y_ == other.y_;
    }

    /**
     * Tests this vector for inequality with the specified vector.
     *
     * @param other  The vector with which to test for inequality
     * @return  True if the vectors are not equal.
     */
    bool operator!=(const Vector2 &other) const
    {
        return !(*this == other);
    }

    /**
     * Sets the value of this vector to the scalar multiplication of itself with the
     * specified scalar value.
     *
     * @param scalar  The scalar value with which the scalar multiplication should be
     * computed.
     * @return  A reference to this vector.
     */
    Vector2 &operator*=(float scalar)
    {
        x_ *= scalar;
        y_ *= scalar;

        return *this;
    }

    /**
     * Sets the value of this vector to the scalar division of itself with the
     * specified scalar value.
     *
     * @param scalar  The scalar value with which the scalar division should be computed.
     * @return  A reference to this vector.
     */
    Vector2 &operator/=(float scalar)
    {
        const float invScalar = 1.0f / scalar;

        x_ *= invScalar;
        y_ *= invScalar;

        return *this;
    }

    /**
     * Sets the value of this vector to the vector sum of itself with the
     * specified vector.
     *
     * @param other  The vector with which the vector sum should be computed.
     * @return  A reference to this vector.
     */
    Vector2 &operator+=(const Vector2 &other)
    {
        x_ += other.x_;
        y_ += other.y_;

        return *this;
    }

    /**
     * Sets the value of this vector to the vector difference of itself with the
     * specified vector.
     *
     * @param other  The vector with which the vector difference should be computed.
     * @return  A reference to this vector.
     */
    Vector2 &operator-=(const Vector2 &other)
    {
        x_ -= other.x_;
        y_ -= other.y_;

        return *this;
    }

   private:
    float x_;
    float y_;
};

/**
 * Computes the length of a specified vector.
 *
 * @param vector  The vector whose length is to be computed.
 * @return  The length of the vector.
 */
inline float abs(const Vector2 &vector)
{
    return std::sqrt(vector * vector);
}

/**
 * Computes the squared length of a specified vector.
 *
 * @param vector  The vector whose squared length is to be calculated.
 * @return  The squared length of the vector.
 */
inline float absSq(const Vector2 &vector)
{
    return vector * vector;
}

/**
 * Computes the angle between a specified vector and the positive x-axis.
 *
 * @param vector  The vector whose angle with the positive x-axis is to be calculated.
 * @return  The angle in radians between the vector and the positive x-axis in the
 * range [-PI, PI].
 */
inline float atan(const Vector2 &vector)
{
    return std::atan2(vector.getY(), vector.getX());
}

/**
 * Computes the determinant of a square matrix with rows consisting of the specified
 * vectors.
 *
 * @param vector1  The top row of the square matrix.
 * @param vector2  The bottom row of the square matrix.
 * @return  The determinant of the square matrix.
 */
inline float det(const Vector2 &vector1, const Vector2 &vector2)
{
    return vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
}

/**
 * Computes the normalization of a specified vector.
 *
 * @param vector  The vector whose normalization is to be calculated.
 * @return  The normalization of the vector.
 */
inline Vector2 normalize(const Vector2 &vector)
{
    // TODO: Added safety check for SIGFPE
    float magnitude = abs(vector);
    if (magnitude != 0)
    {
        return vector / magnitude;
    }
    return Vector2();
}

/**
 * Computes the normal to a line segment with the specified end points.
 *
 * @param vector1  The first end point of the line segment.
 * @param vector2  The second end point of the line segment.
 * @return  The normal vector of the line segment.
 */
inline Vector2 normal(const Vector2 &vector1, const Vector2 &vector2)
{
    return normalize(
        Vector2(vector2.getY() - vector1.getY(), vector1.getX() - vector2.getX()));
}

/**
 * Computes the scalar multiplication of the specified vector with the
 * specified scalar value.
 *
 * @param scalar  The scalar value with which the scalar multiplication should be
 * computed.
 * @param vector  The vector with which the scalar multiplication should be computed.
 * @return  The scalar multiplication of the vector with the scalar value.
 */
inline Vector2 operator*(float scalar, const Vector2 &vector)
{
    return Vector2(scalar * vector.getX(), scalar * vector.getY());
}

/**
 * Inserts the specified two-dimensional vector into the specified output stream.
 *
 * @param stream  The output stream into which the two-dimensional vector should be
 * inserted.
 * @param vector  The two-dimensional vector which to insert into the output stream.
 * @return  A reference to the output stream.
 */
std::ostream &operator<<(std::ostream &stream, const Vector2 &vector);
