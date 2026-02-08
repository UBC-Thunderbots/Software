#pragma once

#include <cmath>
#include <ostream>
#include <type_traits>

#include "software/geom/geom_constants.h"

struct AngleTag;
struct AngularVelocityTag;
struct AngularAccelerationTag;

template <typename Tag>
concept IsBaseAngleTag = std::is_same_v<Tag, AngleTag>;

/**
 * A typesafe representation of an angle.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * without proper conversion.
 */
template <typename Tag>
class GenericAngle final
{
   public:
    /**
     * The zero angle.
     */
    static constexpr GenericAngle zero();

    /**
     * The quarter-turn angle (90°).
     */
    static constexpr GenericAngle quarter();

    /**
     * The half-turn angle (180°).
     */
    static constexpr GenericAngle half();

    /**
     * The three-quarter turn angle (270°).
     */
    static constexpr GenericAngle threeQuarter();

    /**
     * The full-turn angle (360°).
     */
    static constexpr GenericAngle full();

    /**
     * Constructs an angle from a value in radians.
     *
     * @param rad the angle in radians.
     *
     * @return the constructed angle
     */
    static constexpr GenericAngle fromRadians(double rad);

    /**
     * Constructs an angle from a value in degrees.
     *
     * @param deg the angle in degrees
     *
     * @return the constructed angle
     */
    static constexpr GenericAngle fromDegrees(double deg);

    /**
     * Computes the arcsine of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static GenericAngle asin(double x);

    /**
     * Computes the arccosine of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static GenericAngle acos(double x);

    /**
     * Computes the arctangent of a value.
     *
     * @param x the value.
     *
     * @return the angle.
     */
    static GenericAngle atan(double x);

    /**
     * Constructs the "zero" angle.
     */
    explicit constexpr GenericAngle();

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
     * @return the modulus of this GenericAngle ÷ divisor.
     */
    constexpr GenericAngle mod(GenericAngle divisor) const requires IsBaseAngleTag<Tag>;

    /**
     * Computes the remainder of a division between this angle and
     * another angle.
     *
     * @param divisor the divisor.
     *
     * @return the remainder of this GenericAngle ÷ divisor.
     */
    constexpr GenericAngle remainder(const GenericAngle& divisor) const requires
        IsBaseAngleTag<Tag>;

    /**
     * Returns the absolute value of this angle.
     *
     * @return the absolute value of this angle.
     */
    constexpr GenericAngle abs() const;

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
    constexpr GenericAngle clamp() const requires IsBaseAngleTag<Tag>;

    /**
     * Returns the smallest possible rotational difference between this angle
     * and another angle.
     *
     * @param other the second angle.
     *
     * @return the angle between this GenericAngle and other, in the range [0, π].
     */
    constexpr GenericAngle minDiff(const GenericAngle& other) const requires
        IsBaseAngleTag<Tag>;

   private:
    /**
     * The measurement in radians of this GenericAngle.
     */
    double rads;

    explicit constexpr GenericAngle(double rads);
};

using Angle = GenericAngle<AngleTag>;

template <typename T>
concept AngleType = std::is_same_v<T, GenericAngle<AngleTag>> ||
    std::is_same_v<T, GenericAngle<AngularVelocityTag>> ||
    std::is_same_v<T, GenericAngle<AngularAccelerationTag>>;

/**
 * Negates an angle.
 *
 * @param angle the angle to negate.
 *
 * @return the negated angle
 */
constexpr auto operator-(const AngleType auto& angle) __attribute__((warn_unused_result));

/**
 * Adds two angles.
 *
 * @param x the first addend.
 * @param y the second addend.
 *
 * @return the sum of the angles
 */
constexpr auto operator+(const AngleType auto& x, const AngleType auto& y)
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
constexpr auto operator-(const AngleType auto& x, const AngleType auto& y)
    __attribute__((warn_unused_result));

/**
 * Multiplies an angle by a scalar factor.
 *
 * @param angle the angle.
 * @param scale the scalar factor.
 *
 * @return the product of the angle and the scalar factor
 */
template <typename Tag>
constexpr GenericAngle<Tag> operator*(const GenericAngle<Tag>& angle, double scale)
    __attribute__((warn_unused_result));

/**
 * Multiplies an angle by a scalar factor.
 *
 * @param scale the scalar factor.
 * @param angle the angle.
 *
 * @return the product of the angle and the scalar factor
 */
template <typename Tag>
constexpr GenericAngle<Tag> operator*(double scale, const GenericAngle<Tag>& angle)
    __attribute__((warn_unused_result));

/**
 * Divides an angle by a scalar divisor.
 *
 * @param angle the angle.
 * @param divisor the scalar divisor.
 *
 * @return the quotient of this AngleType auto ÷ the divisor.
 */
template <typename Tag>
constexpr GenericAngle<Tag> operator/(const GenericAngle<Tag>& angle, double divisor)
    __attribute__((warn_unused_result));

/**
 * Divides two angles.
 *
 * @param x the divident.
 * @param y the divisor.
 *
 * @return the quotient of the divident ÷ the divisor.
 */
constexpr double operator/(const AngleType auto& x, const AngleType auto& y)
    __attribute__((warn_unused_result));

/**
 * Adds an angle to another angle.
 *
 * @param x the angle to add to.
 * @param y the angle to add.
 *
 * @return the new angle x
 */
auto& operator+=(AngleType auto& x, const AngleType auto& y);

/**
 * Subtracts an angle from an angle.
 *
 * @param x the angle to subtract from.
 * @param y the angle to subtract.
 *
 * @return the new angle x
 */
auto& operator-=(AngleType auto& x, const AngleType auto& y);

/**
 * Scales an angle by a factor.
 *
 * @param angle the angle to scale.
 * @param scale the scalar factor.
 *
 * @return the scaled angle.
 */
auto& operator*=(AngleType auto& angle, double scale);

/**
 * Divides an angle by a scalar divisor.
 *
 * @param angle the angle to scale.
 *
 * @param divisor the scalar divisor.
 *
 * @return the scaled angle.
 */
auto& operator/=(AngleType auto& angle, double divisor);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 *
 * @param y the second angle.
 *
 * @return true if x is strictly less than y, and false otherwise
 */
constexpr bool operator<(const AngleType auto& x, const AngleType auto& y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is strictly greater than y, and false otherwise.
 */
constexpr bool operator>(const AngleType auto& x, const AngleType auto& y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is less than or equal to y, and false otherwise.
 */
constexpr bool operator<=(const AngleType auto& x, const AngleType auto& y);

/**
 * Compares two angles.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is greater than or equal to y, and false otherwise.
 */
constexpr bool operator>=(const AngleType auto& x, const AngleType auto& y);

/**
 * Compares two angles for equality
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is equal to y, and false otherwise.
 */
bool operator==(const AngleType auto& x, const AngleType auto& y);

/**
 * Compares two angles for inequality.
 *
 * @param x the first angle.
 * @param y the second angle.
 *
 * @return true if x is not equal to y, and false otherwise
 */
constexpr bool operator!=(const AngleType auto& x, const AngleType auto& y);

/**
 * Prints an AngleType auto to a stream
 *
 * @param os the stream to print to
 * @param a the Point to print
 *
 * @return the stream with the AngleType auto printed
 */
inline std::ostream& operator<<(std::ostream& os, const AngleType auto& a);

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::zero()
{
    return GenericAngle<Tag>();
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::quarter()
{
    return GenericAngle<Tag>(M_PI / 2.0);
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::half()
{
    return GenericAngle<Tag>(M_PI);
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::threeQuarter()
{
    return GenericAngle<Tag>(3.0 / 2.0 * M_PI);
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::full()
{
    return GenericAngle<Tag>(2.0 * M_PI);
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::fromRadians(double rad)
{
    return GenericAngle<Tag>(rad);
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::fromDegrees(double deg)
{
    return GenericAngle<Tag>(deg / 180.0 * M_PI);
}

template <typename Tag>
inline GenericAngle<Tag> GenericAngle<Tag>::asin(double x)
{
    return GenericAngle<Tag>::fromRadians(std::asin(x));
}

template <typename Tag>
inline GenericAngle<Tag> GenericAngle<Tag>::acos(double x)
{
    return fromRadians(std::acos(x));
}

template <typename Tag>
inline GenericAngle<Tag> GenericAngle<Tag>::atan(double x)
{
    return GenericAngle<Tag>::fromRadians(std::atan(x));
}

template <typename Tag>
inline constexpr GenericAngle<Tag>::GenericAngle() : rads(0.0)
{
}

template <typename Tag>
inline constexpr double GenericAngle<Tag>::toRadians() const
{
    return rads;
}

template <typename Tag>
inline constexpr double GenericAngle<Tag>::toDegrees() const
{
    return rads / M_PI * 180.0;
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::mod(
    GenericAngle<Tag> divisor) const requires IsBaseAngleTag<Tag>
{
    if (divisor.toRadians() < FIXED_EPSILON)
    {
        return GenericAngle<Tag>::fromRadians(toRadians());
    }
    else
    {
        return GenericAngle<Tag>::fromRadians(
            toRadians() -
            static_cast<double>(static_cast<long>(toRadians() / divisor.toRadians())) *
                divisor.toRadians());
    }
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::remainder(
    const GenericAngle<Tag>& divisor) const requires IsBaseAngleTag<Tag>

{
    return GenericAngle<Tag>::fromRadians(
        toRadians() - static_cast<double>(static_cast<long>(
                          (toRadians() / divisor.toRadians()) >= 0
                              ? (toRadians() / divisor.toRadians() + 0.5)
                              : (toRadians() / divisor.toRadians() - 0.5))) *
                          divisor.toRadians());
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::abs() const
{
    return GenericAngle<Tag>::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}

template <typename Tag>
inline bool GenericAngle<Tag>::isFinite() const
{
    return std::isfinite(toRadians());
}

template <typename Tag>
inline double GenericAngle<Tag>::sin() const
{
    return std::sin(toRadians());
}

template <typename Tag>
inline double GenericAngle<Tag>::cos() const
{
    return std::cos(toRadians());
}

template <typename Tag>
inline double GenericAngle<Tag>::tan() const
{
    return std::tan(toRadians());
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::clamp() const requires
    IsBaseAngleTag<Tag>

{
    return remainder(GenericAngle<Tag>::full());
}

template <typename Tag>
inline constexpr GenericAngle<Tag> GenericAngle<Tag>::minDiff(
    const GenericAngle<Tag>& other) const requires IsBaseAngleTag<Tag>

{
    return (*this - other).clamp().abs();
}

template <typename Tag>
inline constexpr GenericAngle<Tag>::GenericAngle(double rads) : rads(rads)
{
}

inline constexpr auto operator-(const AngleType auto& angle)
{
    using T = std::remove_cvref_t<decltype(angle)>;
    return T::fromRadians(-angle.toRadians());
}

inline constexpr auto operator+(const AngleType auto& x, const AngleType auto& y)
{
    using T = std::remove_cvref_t<decltype(x)>;
    return T::fromRadians(x.toRadians() + y.toRadians());
}

inline constexpr auto operator-(const AngleType auto& x, const AngleType auto& y)
{
    using T = std::remove_cvref_t<decltype(x)>;
    return T::fromRadians(x.toRadians() - y.toRadians());
}

template <typename Tag>
inline constexpr GenericAngle<Tag> operator*(const GenericAngle<Tag>& angle, double scale)
{
    return GenericAngle<Tag>::fromRadians(angle.toRadians() * scale);
}

template <typename Tag>
inline constexpr GenericAngle<Tag> operator*(double scale, const GenericAngle<Tag>& angle)
{
    return GenericAngle<Tag>::fromRadians(scale * angle.toRadians());
}

template <typename Tag>
inline constexpr GenericAngle<Tag> operator/(const GenericAngle<Tag>& angle,
                                             double divisor)
{
    return GenericAngle<Tag>::fromRadians(angle.toRadians() / divisor);
}

inline constexpr double operator/(const AngleType auto& x, const AngleType auto& y)
{
    return x.toRadians() / y.toRadians();
}

inline auto& operator+=(AngleType auto& x, const AngleType auto& y)
{
    return x = x + y;
}

inline auto& operator-=(AngleType auto& x, const AngleType auto& y)
{
    return x = x - y;
}

inline auto& operator*=(AngleType auto& angle, double scale)
{
    return angle = angle * scale;
}

inline auto& operator/=(AngleType auto& angle, double divisor)
{
    return angle = angle / divisor;
}

inline constexpr bool operator<(const AngleType auto& x, const AngleType auto& y)
{
    return x.toRadians() < y.toRadians();
}

inline constexpr bool operator>(const AngleType auto& x, const AngleType auto& y)
{
    return x.toRadians() > y.toRadians();
}

inline constexpr bool operator<=(const AngleType auto& x, const AngleType auto& y)
{
    return x.toRadians() <= y.toRadians();
}

inline constexpr bool operator>=(const AngleType auto& x, const AngleType auto& y)
{
    return x.toRadians() >= y.toRadians();
}

inline bool operator==(const AngleType auto& x, const AngleType auto& y)
{
    AngleType auto diff = x.clamp().minDiff(y.clamp());
    return diff.toRadians() <= FIXED_EPSILON;
}

inline constexpr bool operator!=(const AngleType auto& x, const AngleType auto& y)
{
    return x.toRadians() != y.toRadians();
}

inline std::ostream& operator<<(std::ostream& os, const AngleType auto& a)
{
    os << a.toRadians() << "R";
    return os;
}
