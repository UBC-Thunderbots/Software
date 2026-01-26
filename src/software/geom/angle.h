#pragma once

#include "software/geom/generic_angle.h"
#include "software/geom/geom_constants.h"

class Angle : public GenericAngle<Angle>
{
   public:
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
     * Limits this angle to [−π, π].
     *
     * The angle is rotated by a multiple of 2π until it lies within the target
     * interval.
     *
     * @return the clamped angle.
     */
    constexpr Angle clamp() const;

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
    constexpr Angle remainder(const Angle& divisor) const;

    constexpr Angle minDiff(const Angle& other) const;

    bool operator==(const Angle& other) const;

    constexpr Angle() = default;
    explicit constexpr Angle(double rad);
};

inline constexpr Angle::Angle(double rad) : GenericAngle(rad) {}

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

inline constexpr Angle Angle::clamp() const
{
    return remainder(Angle::full());
}

inline constexpr Angle Angle::remainder(const Angle& divisor) const
{
    return Angle::fromRadians(toRadians() -
                              static_cast<double>(static_cast<long>(
                                  (toRadians() / divisor.toRadians()) >= 0
                                      ? (toRadians() / divisor.toRadians() + 0.5)
                                      : (toRadians() / divisor.toRadians() - 0.5))) *
                                  divisor.toRadians());
}

inline constexpr Angle Angle::mod(Angle divisor) const
{
    if (divisor.toRadians() < FIXED_EPSILON)
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

inline constexpr Angle Angle::minDiff(const Angle& other) const
{
    return (*this - other).clamp().abs();
}

inline bool Angle::operator==(const Angle& other) const
{
    Angle diff = this->clamp().minDiff(other.clamp());
    return diff.toRadians() <= FIXED_EPSILON;
};
