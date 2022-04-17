#pragma once

#include <cmath>
#include <ostream>

#include "software/geom/geom_constants.h"

/**
 * A typesafe representation of an angle.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * without proper conversion.
 */
class Angle final
{
   public:
    /**
     * The zero angle.
     */
    static constexpr Angle zero();

    /**
     * The quarter-turn angle (90°).
     */
    static constexpr Angle quarter();

    /**
     * The half-turn angle (180°).
     */
    static constexpr Angle half();

    /**
     * The three-quarter turn angle (270°).
     */
    static constexpr Angle threeQuarter();

    /**
     * The full-turn angle (360°).
     */
    static constexpr Angle full();

    /**
     * Constructs an angle from a value in radians.
     *
     * @param rad the angle in radians.
     *
     * @return the constructed angle
     */
    static constexpr Angle fromRadians(double rad);

    /**
     * Constructs an angle from a value in degrees.
     *
     * @param deg the angle in degrees
     *
     * @return the constructed angle
     */
    static constexpr Angle fromDegrees(double deg);

    /**
     * Computes the arcsine of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static Angle asin(double x);

    /**
     * Computes the arccosine of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static Angle acos(double x);

    /**
     * Computes the arctangent of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static Angle atan(double x);

    /**
     * Constructs the "zero" angle.
     */
    explicit constexpr Angle();

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
     * Computes the modulus of a division between this angle and another
     * angle.
     *
     * @param divisor the divisor.
     *
     * @return the modulus of this Angle ÷ divisor.
     */
    constexpr Angle mod(Angle divisor) const;

    /**
     * Computes the remainder of a division between this angle and
     * another angle.
     *
     * @param divisor the divisor.
     *
     * @return the remainder of this Angle ÷ divisor.
     */
    constexpr Angle remainder(const Angle &divisor) const;

    /**
     * Returns the absolute value of this angle.
     *
     * @return the absolute value of this angle.
     */
    constexpr Angle abs() const;

    /**
     * Checks whether the angle is finite.
     *
     * @return true if the angle is finite, and false if it is ±∞ or NaN.
     */
    bool isFinite() const;

    /**
     * Computes the sine of this angle.
     *
     * @return the sine of this angle.
     */
    double sin() const;

    /**
     * Computes the cosine of this angle.
     *
     * @return the cosine of this angle.
     */
    double cos() const;

    /**
     * Computes the tangent of this angle.
     *
     * @return the tangent of this angle.
     */
    double tan() const;

    /**
     * Limits this angle to [−π, π].
     *
     * The angle is rotated by a multiple of 2π until it lies within the target
     * interval.
     *
     * @return the clamped angle.
     */
    constexpr Angle clamp() const;

    /**
     * Returns the smallest possible rotational difference between this angle
     * and another angle.
     *
     * @param other the second angle.
     *
     * @return the angle between this Angle and other, in the range [0, π].
     */
    constexpr Angle minDiff(const Angle &other) const;

   private:
    /**
     * The measurement in radians of this Angle.
     */
    double rads;

    explicit constexpr Angle(double rads);
};

/**
 * Negates an angle.
 *
 * @param angle the angle to negate.
 *
 * @return the negated angle
 */
constexpr Angle operator-(const Angle &angle) __attribute__((warn_unused_result));

/**
 * Adds two angles.
 *
 * @param x the first addend.
 * @param y the second addend.
 *
 * @return the sum of the angles
 */
constexpr Angle operator+(const Angle &x, const Angle &y)
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
constexpr Angle operator-(const Angle &x, const Angle &y)
    __attribute__((warn_unused_result));

/**
 * Multiplies an angle by a scalar factor.
 *
 * @param angle the angle.
 * @param scale the scalar factor.
 *
 * @return the product of the angle and the scalar factor
 */
constexpr Angle operator*(const Angle &angle, double scale)
    __attribute__((warn_unused_result));

/**
 * Multiplies an angle by a scalar factor.
 *
 * @param scale the scalar factor.
 * @param angle the angle.
 *
 * @return the product of the angle and the scalar factor
 */
constexpr Angle operator*(double scale, const Angle &angle)
    __attribute__((warn_unused_result));

/**
 * Divides an angle by a scalar divisor.
 *
 * @param angle the angle.
 * @param divisor the scalar divisor.
 *
 * @return the quotient of this Angle ÷ the divisor.
 */
constexpr Angle operator/(const Angle &angle, double divisor)
    __attribute__((warn_unused_result));

/**
 * Divides two angles.
 *
 * @param x the divident.
 * @param y the divisor.
 *
 * @return the quotient of the divident ÷ the divisor.
 */
constexpr double operator/(const Angle &x, const Angle &y)
    __attribute__((warn_unused_result));

/**
 * Adds an angle to another angle.
 *
 * @param x the angle to add to.
 * @param y the angle to add.
 *
 * @return the new angle x
 */
Angle &operator+=(Angle &x, const Angle &y);

/**
 * Subtracts an angle from an angle.
 *
 * @param x the angle to subtract from.
 * @param y the angle to subtract.
 *
 * @return the new angle x
 */
Angle &operator-=(Angle &x, const Angle &y);

/**
 * Scales an angle by a factor.
 *
 * @param angle the angle to scale.
 * @param scale the scalar factor.
 *
 * @return the scaled angle.
 */
Angle &operator*=(Angle &angle, double scale);

/**
 * Divides an angle by a scalar divisor.
 *
 * @param angle the angle to scale.
 *
 * @param divisor the scalar divisor.
 *
 * @return the scaled angle.
 */
Angle &operator/=(Angle &angle, double divisor);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 *
 * @param y the second angle.
 *
 * @return true if x is strictly less than y, and false otherwise
 */
constexpr bool operator<(const Angle &x, const Angle &y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is strictly greater than y, and false otherwise.
 */
constexpr bool operator>(const Angle &x, const Angle &y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is less than or equal to y, and false otherwise.
 */
constexpr bool operator<=(const Angle &x, const Angle &y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is greater than or equal to y, and false otherwise.
 */
constexpr bool operator>=(const Angle &x, const Angle &y);

/**
 * Compares two angles for equality
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is equal to y, and false otherwise.
 */
bool operator==(const Angle &x, const Angle &y);

/**
 * Compares two angles for inequality.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is not equal to y, and false otherwise
 */
constexpr bool operator!=(const Angle &x, const Angle &y);

/**
 * Prints an Angle to a stream
 *
 * @param os the stream to print to
 * @param a the Point to print
 *
 * @return the stream with the Angle printed
 */
inline std::ostream &operator<<(std::ostream &os, const Angle &a);

inline constexpr Angle Angle::zero()
{
    return Angle();
}

inline constexpr Angle Angle::quarter()
{
    return Angle(M_PI / 2.0);
}

inline constexpr Angle Angle::half()
{
    return Angle(M_PI);
}

inline constexpr Angle Angle::threeQuarter()
{
    return Angle(3.0 / 2.0 * M_PI);
}

inline constexpr Angle Angle::full()
{
    return Angle(2.0 * M_PI);
}

inline constexpr Angle Angle::fromRadians(double rad)
{
    return Angle(rad);
}

inline constexpr Angle Angle::fromDegrees(double deg)
{
    return Angle(deg / 180.0 * M_PI);
}

inline Angle Angle::asin(double x)
{
    return Angle::fromRadians(std::asin(x));
}

inline Angle Angle::acos(double x)
{
    return fromRadians(std::acos(x));
}

inline Angle Angle::atan(double x)
{
    return Angle::fromRadians(std::atan(x));
}

inline constexpr Angle::Angle() : rads(0.0) {}

inline constexpr double Angle::toRadians() const
{
    return rads;
}

inline constexpr double Angle::toDegrees() const
{
    return rads / M_PI * 180.0;
}

inline constexpr Angle Angle::mod(Angle divisor) const
{
    if (divisor.toRadians() == 0)
    {
        return Angle::fromRadians(toRadians());
    }
    else
    {
        return Angle::fromRadians(toRadians() - static_cast<double>(static_cast<long>(
                                                    toRadians() / divisor.toRadians())) *
                                                    divisor.toRadians());
    }
}

inline constexpr Angle Angle::remainder(const Angle &divisor) const
{
    return Angle::fromRadians(toRadians() -
                              static_cast<double>(static_cast<long>(
                                  (toRadians() / divisor.toRadians()) >= 0
                                      ? (toRadians() / divisor.toRadians() + 0.5)
                                      : (toRadians() / divisor.toRadians() - 0.5))) *
                                  divisor.toRadians());
}

inline constexpr Angle Angle::abs() const
{
    return Angle::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}

inline bool Angle::isFinite() const
{
    return std::isfinite(toRadians());
}

inline double Angle::sin() const
{
    return std::sin(toRadians());
}

inline double Angle::cos() const
{
    return std::cos(toRadians());
}

inline double Angle::tan() const
{
    return std::tan(toRadians());
}

inline constexpr Angle Angle::clamp() const
{
    return remainder(Angle::full());
}

inline constexpr Angle Angle::minDiff(const Angle &other) const
{
    return (*this - other).clamp().abs();
}

inline constexpr Angle::Angle(double rads) : rads(rads) {}

inline constexpr Angle operator-(const Angle &angle)
{
    return Angle::fromRadians(-angle.toRadians());
}

inline constexpr Angle operator+(const Angle &x, const Angle &y)
{
    return Angle::fromRadians(x.toRadians() + y.toRadians());
}

inline constexpr Angle operator-(const Angle &x, const Angle &y)
{
    return Angle::fromRadians(x.toRadians() - y.toRadians());
}

inline constexpr Angle operator*(const Angle &angle, double scale)
{
    return Angle::fromRadians(angle.toRadians() * scale);
}

inline constexpr Angle operator*(double scale, const Angle &angle)
{
    return Angle::fromRadians(scale * angle.toRadians());
}

inline constexpr Angle operator/(const Angle &angle, double divisor)
{
    return Angle::fromRadians(angle.toRadians() / divisor);
}

inline constexpr double operator/(const Angle &x, const Angle &y)
{
    return x.toRadians() / y.toRadians();
}

inline Angle &operator+=(Angle &x, const Angle &y)
{
    return x = x + y;
}

inline Angle &operator-=(Angle &x, const Angle &y)
{
    return x = x - y;
}

inline Angle &operator*=(Angle &angle, double scale)
{
    return angle = angle * scale;
}

inline Angle &operator/=(Angle &angle, double divisor)
{
    return angle = angle / divisor;
}

inline constexpr bool operator<(const Angle &x, const Angle &y)
{
    return x.toRadians() < y.toRadians();
}

inline constexpr bool operator>(const Angle &x, const Angle &y)
{
    return x.toRadians() > y.toRadians();
}

inline constexpr bool operator<=(const Angle &x, const Angle &y)
{
    return x.toRadians() <= y.toRadians();
}

inline constexpr bool operator>=(const Angle &x, const Angle &y)
{
    return x.toRadians() >= y.toRadians();
}

inline bool operator==(const Angle &x, const Angle &y)
{
    Angle diff = x.clamp().minDiff(y.clamp());
    return diff.toRadians() <= FIXED_EPSILON;
}

inline constexpr bool operator!=(const Angle &x, const Angle &y)
{
    return x.toRadians() != y.toRadians();
}

inline std::ostream &operator<<(std::ostream &os, const Angle &a)
{
    os << a.toRadians() << "R";
    return os;
}
