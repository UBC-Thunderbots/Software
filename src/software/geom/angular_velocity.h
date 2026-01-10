#pragma once

#include "software/geom/angle.h"

/**
 * A typesafe representation of an angular velocity, using the Angle class.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * per second without proper conversion.
 *
 */
class AngularVelocity : public Angle
{
   public:
    /**
     * Constructs angular velocity of zero.
     */
    explicit constexpr AngularVelocity();

    /**
     * Constructs angular velocity from an angle, converting unit of angle to a change in
     * angle per second.
     */
    explicit constexpr AngularVelocity(const Angle& angle);

    /**
     * Zero angular velocity.
     */
    static constexpr AngularVelocity zero();

    /**
     * Quarter turn every second (90°/s)
     */
    static constexpr AngularVelocity quarter();

    /**
     * Half turn every second (180°/s)
     */
    static constexpr AngularVelocity half();

    /**
     * Three-quarter turn every second (270°/s)
     */
    static constexpr AngularVelocity threeQuarter();

    /**
     * Full turn every second (360°/s)
     */
    static constexpr AngularVelocity full();

    /**
     * Constructs an angular velocity from a value in radians per second.
     *
     * @param rad the angular velocity in radians per second.
     *
     * @return the constructed angular velocity
     */
    static constexpr AngularVelocity fromRadians(double rad);

    /**
     * Constructs an angular velocity from a value in degrees per second.
     *
     * @param deg the angular velocity in degrees per second.
     *
     * @return the constructed angular velocity
     */
    static constexpr AngularVelocity fromDegrees(double deg);

    /**
     * Returns the absolute value of this angular velocity.
     *
     * @return the absolute value of this angular velocity.
     */
    constexpr AngularVelocity abs() const;

    /**
     * Returns the rotational velocity difference between this angular velocity and
     * another angular velocity
     *
     * @param other the second angular velocity.
     *
     * @return the absolute difference in angular velocity between this angular velocity
     * and other
     */
    constexpr AngularVelocity minDiff(const AngularVelocity& other);

    // Delete methods that do not apply to angular velocity
    static Angle asin(double)                     = delete;
    static Angle acos(double)                     = delete;
    static Angle atan(double)                     = delete;
    constexpr Angle mod(Angle) const              = delete;
    constexpr Angle remainder(const Angle&) const = delete;
    double sin() const                            = delete;
    double cos() const                            = delete;
    double tan() const                            = delete;
    constexpr Angle clamp() const                 = delete;
    constexpr Angle minDiff(const Angle&) const   = delete;
};

inline constexpr AngularVelocity::AngularVelocity() : Angle(0.0) {}

inline constexpr AngularVelocity::AngularVelocity(const Angle& angle) : Angle(angle) {}

inline constexpr AngularVelocity AngularVelocity::zero()
{
    return AngularVelocity(Angle::zero());
}

inline constexpr AngularVelocity AngularVelocity::quarter()
{
    return AngularVelocity(Angle::quarter());
}

inline constexpr AngularVelocity AngularVelocity::half()
{
    return AngularVelocity(Angle::half());
}

inline constexpr AngularVelocity AngularVelocity::threeQuarter()
{
    return AngularVelocity(Angle::threeQuarter());
}

inline constexpr AngularVelocity AngularVelocity::full()
{
    return AngularVelocity(Angle::full());
}

inline constexpr AngularVelocity AngularVelocity::fromRadians(double rad)
{
    return AngularVelocity(Angle::fromRadians(rad));
}

inline constexpr AngularVelocity AngularVelocity::fromDegrees(double deg)
{
    return AngularVelocity(Angle::fromDegrees(deg));
}

inline constexpr AngularVelocity AngularVelocity::abs() const
{
    return AngularVelocity::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}

inline constexpr AngularVelocity AngularVelocity::minDiff(const AngularVelocity& other)
{
    return AngularVelocity::fromRadians(std::fabs(this->toRadians() - other.toRadians()));
}
