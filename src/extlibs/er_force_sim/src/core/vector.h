/***************************************************************************
 *   Copyright 2015 Philipp Nordhus                                        *
 *   Robotics Erlangen e.V.                                                *
 *   http://www.robotics-erlangen.de/                                      *
 *   info@robotics-erlangen.de                                             *
 *                                                                         *
 *   This program is free software: you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation, either version 3 of the License, or     *
 *   any later version.                                                    *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef VECTOR_H
#define VECTOR_H

#include <cmath>
#include <ostream>

/*!
 * \ingroup path
 * \brief 2-dimensional vector class
 */
class ErForceVector
{
   public:
    /*!
     * \brief Contructor
     *
     * This constructor does not initialize any members
     */
    ErForceVector() {}

    /*!
     * \brief Initializing constructor
     * \param x x-coordinate of the vector
     * \param y y-coordinate of the vector
     */
    ErForceVector(float x, float y)
    {
        d[0] = x;
        d[1] = y;
    }

   public:
    float &operator[](unsigned int index);
    float operator[](unsigned int index) const;
    ErForceVector operator+(const ErForceVector &rho) const;
    ErForceVector operator-(const ErForceVector &rho) const;
    ErForceVector operator*(float scalar) const;
    ErForceVector operator/(float scalar) const;
    ErForceVector &operator*=(float scalar);
    ErForceVector &operator+=(const ErForceVector &other);
    float operator*(const ErForceVector &rho) const;
    bool operator==(const ErForceVector &rho) const;
    bool operator!=(const ErForceVector &rho) const;

    ErForceVector perpendicular() const;
    ErForceVector normalized() const;
    float length() const;
    float lengthSquared() const;
    float distance(const ErForceVector &rho) const;
    float distanceSq(const ErForceVector &rho) const
    {
        return (*this - rho).lengthSquared();
    }

    float dot(const ErForceVector &other) const
    {
        return x * other.x + y * other.y;
    }

    static float det(const ErForceVector &a, const ErForceVector &b,
                     const ErForceVector &c)
    {
        return a.x * b.y + b.x * c.y + c.x * a.y - a.x * c.y - b.x * a.y - c.x * b.y;
    }

    // WARNING: behaves differently than in typescript
    float angle() const
    {
        return std::atan2(x, y) + float(2 * M_PI);
    }

    friend std::ostream &operator<<(std::ostream &stream, const ErForceVector v);

   public:
    union
    {
        struct
        {
            float x;
            float y;
        };
        float d[2];
    };
};

inline float &ErForceVector::operator[](unsigned int index)
{
    return d[index];
}

inline float ErForceVector::operator[](unsigned int index) const
{
    return d[index];
}

inline ErForceVector ErForceVector::operator+(const ErForceVector &rho) const
{
    return ErForceVector(x + rho.x, y + rho.y);
}

inline ErForceVector ErForceVector::operator-(const ErForceVector &rho) const
{
    return ErForceVector(x - rho.x, y - rho.y);
}

inline ErForceVector ErForceVector::operator*(float scalar) const
{
    return ErForceVector(x * scalar, y * scalar);
}

inline ErForceVector ErForceVector::operator/(float scalar) const
{
    return ErForceVector(x / scalar, y / scalar);
}

inline float ErForceVector::operator*(const ErForceVector &rho) const
{
    return x * rho.x + y * rho.y;
}

inline bool ErForceVector::operator==(const ErForceVector &rho) const
{
    return x == rho.x && y == rho.y;
}

inline bool ErForceVector::operator!=(const ErForceVector &rho) const
{
    return x != rho.x || y != rho.y;
}

inline ErForceVector &ErForceVector::operator+=(const ErForceVector &other)
{
    x += other.x;
    y += other.y;
    return *this;
}

inline ErForceVector &ErForceVector::operator*=(float scalar)
{
    x *= scalar;
    y *= scalar;
    return *this;
}

/*!
 * \brief Calculate a perpendicular vector
 * Returns perpendicular which is reached first when rotating clockwise.
 * \return A vector perpendicular to this one
 */
inline ErForceVector ErForceVector::perpendicular() const
{
    return ErForceVector(y, -x);
}

/*!
 * \brief Normalize the vector
 * \return A normalized copy of the vector
 */
inline ErForceVector ErForceVector::normalized() const
{
    ErForceVector v = *this;
    const float l   = length();
    if (l > 0)
    {
        v.x /= l;
        v.y /= l;
    }
    return v;
}

/*!
 * \brief Calculate the length of the vector
 * \return The length of the vector
 */
inline float ErForceVector::length() const
{
    return std::sqrt(lengthSquared());
}

/*!
 * \brief Calculate the squared length of the vector
 * \return The squared length of the vector
 */
inline float ErForceVector::lengthSquared() const
{
    return d[0] * d[0] + d[1] * d[1];
}

/*!
 * \brief Calculate the distance to another vector
 * \param rho Another vector
 * \return The distance to the other vector
 */
inline float ErForceVector::distance(const ErForceVector &rho) const
{
    return (*this - rho).length();
}

inline std::ostream &operator<<(std::ostream &stream, const ErForceVector v)
{
    stream << "ErForceVector(" << v.x << ", " << v.y << ")";
    return stream;
}

#endif  // VECTOR_H
