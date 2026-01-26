#pragma once

#include <cmath>
#include <ostream>

#include "software/geom/geom_constants.h"

/**
 * A typesafe representation of a generic angle.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * without proper conversion.
 *
 * @tparam T The type of angle being represented
 */
template <typename Derived>
class GenericAngle
{
   public:
    /**
     * The zero angle.
     */
    static constexpr Derived zero();

    /**
     * The quarter-turn angle (90°).
     */
    static constexpr Derived quarter();

    /**
     * The half-turn angle (180°).
     */
    static constexpr Derived half();

    /**
     * The three-quarter turn angle (270°).
     */
    static constexpr Derived threeQuarter();

    /**
     * The full-turn angle (360°).
     */
    static constexpr Derived full();

    /**
     * Constructs an angle from a value in radians.
     *
     * @param rad the angle in radians.
     *
     * @return the constructed angle
     */
    static constexpr Derived fromRadians(double rad);

    /**
     * Constructs an angle from a value in degrees.
     *
     * @param deg the angle in degrees
     *
     * @return the constructed angle
     */
    static constexpr Derived fromDegrees(double deg);

    /**
     * Converts this angle to a value in radians.
     *
     * @return the number of radians in this angle in the range [0, 2PI)
     */
    constexpr double toRadians() const;

    /**
     * Converts this angle to a value in degrees.
     *
     * @return the number of degrees in this angle in the range [0,360)
     */
    constexpr double toDegrees() const;

    /**
     * Returns the absolute value of this angle.
     *
     * @return the absolute value of this angle.
     */
    constexpr Derived abs() const;

    /**
     * Checks whether the angle is finite.
     *
     * @return true if the angle is finite, and false if it is ±∞ or NaN.
     */
    bool isFinite() const;

    /**
     * Returns the smallest possible rotational difference between this angle
     * and another angle.
     *
     * @param other the second angle.
     *
     * @return the angle between this Angle and other, in the range [0, π].
     */
    constexpr Derived minDiff(const Derived& other) const;

    /**
     * Negates an angle.
     *
     * @param angle the angle to negate.
     *
     * @return the negated angle
     */
    constexpr Derived operator-() const __attribute__((warn_unused_result));

    /**
     * Adds two angles.
     *
     * @param x the first addend.
     * @param y the second addend.
     *
     * @return the sum of the angles
     */
    constexpr Derived operator+(const Derived& other) const
        __attribute__((warn_unused_result));

    /**
     * Subtracts two angles.
     *
     * @param x the minuend.
     *
     * @param y the subtrahend.
     *
     * @return the difference between the minuend and subtrahend.
     */
    constexpr Derived operator-(const Derived& other) const
        __attribute__((warn_unused_result));

    /**
     * Multiplies an angle by a scalar factor.
     *
     * @param angle the angle.
     * @param scale the scalar factor.
     *
     * @return the product of the angle and the scalar factor
     */
    constexpr Derived operator*(double scale) const __attribute__((warn_unused_result));

    /**
     * Multiplies an angle by a scalar factor.
     *
     * @param angle the angle.
     * @param scale the scalar factor.
     *
     * @return the product of the angle and the scalar factor
     */
    // template <typename D>
    // friend constexpr Derived operator*(double scale, const GenericAngle<D>& a);

    /**
     * Divides an angle by a scalar divisor.
     *
     * @param angle the angle.
     * @param divisor the scalar divisor.
     *
     * @return the quotient of this Angle ÷ the divisor.
     */
    constexpr Derived operator/(double divisor) const __attribute__((warn_unused_result));

    /**
     * Divides two angles.
     *
     * @param x the divident.
     * @param y the divisor.
     *
     * @return the quotient of the divident ÷ the divisor.
     */
    constexpr double operator/(const Derived& divisor) const
        __attribute__((warn_unused_result));

    /**
     * Adds an angle to another angle.
     *
     * @param x the angle to add to.
     * @param y the angle to add.
     *
     * @return the new angle x
     */
    Derived& operator+=(const Derived& other);

    /**
     * Subtracts an angle from an angle.
     *
     * @param x the angle to subtract from.
     * @param y the angle to subtract.
     *
     * @return the new angle x
     */
    Derived& operator-=(const Derived& other);

    /**
     * Scales an angle by a factor.
     *
     * @param angle the angle to scale.
     * @param scale the scalar factor.
     *
     * @return the scaled angle.
     */
    Derived& operator*=(double scale);

    /**
     * Divides an angle by a scalar divisor.
     *
     * @param angle the angle to scale.
     *
     * @param divisor the scalar divisor.
     *
     * @return the scaled angle.
     */
    Derived& operator/=(double divisor);

    /**
     * Compares two angles.
     *
     * @param x the first angle.
     *
     * @param y the second angle.
     *
     * @return true if x is strictly less than y, and false otherwise
     */
    constexpr bool operator<(const Derived& other) const;

    /**
     * Compares two angles.
     *
     * @param x the first angle.
     * @param y the second angle.
     *
     * @return true if x is strictly greater than y, and false otherwise.
     */
    constexpr bool operator>(const Derived& other) const;

    /**
     * Compares two angles.
     *
     * @param x the first angle.
     * @param y the second angle.
     *
     * @return true if x is less than or equal to y, and false otherwise.
     */
    constexpr bool operator<=(const Derived& other) const;

    /**
     * Compares two angles.
     *
     * @param x the first angle.
     * @param y the second angle.
     *
     * @return true if x is greater than or equal to y, and false otherwise.
     */
    constexpr bool operator>=(const Derived& other) const;

    /**
     * Compares two angles for equality
     *
     * @param x the first angle.
     * @param y the second angle.
     *
     * @return true if x is equal to y, and false otherwise.
     */
    bool operator==(const Derived& other) const;

    /**
     * Compares two angles for inequality.
     *
     * @param x the first angle.
     * @param y the second angle.
     *
     * @return true if x is not equal to y, and false otherwise
     */
    constexpr bool operator!=(const Derived& other) const;

    /**
     * Prints an Angle to a stream
     *
     * @param os the stream to print to
     * @param a the Point to print
     *
     * @return the stream with the Angle printed
     */
    template <typename D>
    friend std::ostream& operator<<(std::ostream& os, const GenericAngle<D>& a);

   protected:
    /**
     * The measurement in radians of this Angle.
     */
    double rads = 0.0;

    constexpr GenericAngle() = default;
    explicit constexpr GenericAngle(double rad);
};

template <typename T>
inline constexpr GenericAngle<T>::GenericAngle(double rad) : rads(rad)
{
}

template <typename T>
inline constexpr T GenericAngle<T>::zero()
{
    return T::fromRadians(0.0);
}

template <typename T>
inline constexpr T GenericAngle<T>::quarter()
{
    return T::fromRadians(M_PI / 2.0);
}

template <typename T>
inline constexpr T GenericAngle<T>::half()
{
    return T::fromRadians(M_PI);
}

template <typename T>
inline constexpr T GenericAngle<T>::threeQuarter()
{
    return T::fromRadians(3.0 / 2.0 * M_PI);
}

template <typename T>
inline constexpr T GenericAngle<T>::full()
{
    return T::fromRadians(2.0 * M_PI);
}

template <typename T>
inline constexpr T GenericAngle<T>::fromRadians(double rad)
{
    return T(rad);
}

template <typename T>
inline constexpr T GenericAngle<T>::fromDegrees(double deg)
{
    return T::fromRadians(deg / 180.0 * M_PI);
}

template <typename T>
inline constexpr double GenericAngle<T>::toRadians() const
{
    return rads;
}

template <typename T>
inline constexpr double GenericAngle<T>::toDegrees() const
{
    return rads / M_PI * 180.0;
}

template <typename T>
inline constexpr T GenericAngle<T>::abs() const
{
    return T::fromRadians(rads < 0 ? -rads : rads);
}

template <typename T>
inline bool GenericAngle<T>::isFinite() const
{
    return std::isfinite(rads);
}

template <typename T>
inline constexpr T GenericAngle<T>::minDiff(const T& other) const
{
    return T::fromRadians(fabs(toRadians() - other.toRadians()));
}

template <typename T>
inline constexpr T GenericAngle<T>::operator-() const
{
    return T::fromRadians(-rads);
}

template <typename T>
inline constexpr T GenericAngle<T>::operator+(const T& other) const
{
    return T::fromRadians(rads + other.toRadians());
}

template <typename T>
inline constexpr T GenericAngle<T>::operator-(const T& other) const
{
    return T::fromRadians(rads - other.toRadians());
}

template <typename T>
inline constexpr T GenericAngle<T>::operator*(double scale) const
{
    return T::fromRadians(rads * scale);
}

template <typename T>
inline constexpr T GenericAngle<T>::operator/(double divisor) const
{
    return T::fromRadians(rads / divisor);
}

template <typename T>
inline constexpr double GenericAngle<T>::operator/(const T& y) const
{
    return rads / y.toRadians();
}

template <typename T>
inline T& GenericAngle<T>::operator+=(const T& y)
{
    T& self     = static_cast<T&>(*this);
    return self = self + y;
}

template <typename T>
inline T& GenericAngle<T>::operator-=(const T& y)
{
    T& self     = static_cast<T&>(*this);
    return self = self - y;
}

template <typename T>
inline T& GenericAngle<T>::operator*=(double scale)
{
    T& self     = static_cast<T&>(*this);
    return self = self * scale;
}

template <typename T>
inline T& GenericAngle<T>::operator/=(double divisor)
{
    T& self     = static_cast<T&>(*this);
    return self = self / divisor;
}

template <typename T>
inline constexpr bool GenericAngle<T>::operator<(const T& y) const
{
    return rads < y.toRadians();
}

template <typename T>
inline constexpr bool GenericAngle<T>::operator>(const T& y) const
{
    return rads > y.toRadians();
}

template <typename T>
inline constexpr bool GenericAngle<T>::operator<=(const T& y) const
{
    return rads <= y.toRadians();
}

template <typename T>
inline constexpr bool GenericAngle<T>::operator>=(const T& y) const
{
    return rads >= y.toRadians();
}

template <typename T>
inline bool GenericAngle<T>::operator==(const T& y) const
{
    return fabs(rads - y.toRadians()) <= FIXED_EPSILON;
}

template <typename T>
inline constexpr bool GenericAngle<T>::operator!=(const T& y) const
{
    return !(*static_cast<T*>(this) == y);
}

template <typename T>
inline std::ostream& operator<<(std::ostream& os, const GenericAngle<T>& a)
{
    os << a.toRadians() << "R";
    return os;
}

template <typename T>
inline constexpr T operator*(double scale, const GenericAngle<T>& a)
{
    return T::fromRadians(scale * a.toRadians());
}
