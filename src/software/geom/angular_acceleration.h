#pragma once

#include "software/geom/angle.h"

/**
 * A typesafe representation of an angular acceleration, using the Angle class.
 *
 * This class helps prevent accidentally combining values in degrees and radians
 * per seconds squared without proper conversion.
 *
 */
class AngularAcceleration : public Angle
{
   public:
    /**
     * Constructs angular acceleration of zero.
     */
    explicit constexpr AngularAcceleration();

    /**
     * Constructs angular acceleration from an angle, converting unit of angle to a change
     * in angle per seconds squared.
     */
    explicit constexpr AngularAcceleration(const Angle& angle);

    /**
     * Zero angular velocity.
     */
    static constexpr AngularAcceleration zero();

    /**
     * Constructs an angular acceleration from a value in radians per second squared.
     *
     * @param rad the angular acceleration in radians per second squared.
     *
     * @return the constructed angular acceleration
     */
    static constexpr AngularAcceleration fromRadians(double rad);

    /**
     * Constructs an angular acceleration from a value in degrees per second squared.
     *
     * @param rad the angular acceleration in degrees per second squared.
     *
     * @return the constructed angular acceleration
     */
    static constexpr AngularAcceleration fromDegrees(double deg);

    /**
     * Returns the absolute value of this angular acceleration.
     *
     * @return the absolute value of this angular acceleration.
     */
    constexpr AngularAcceleration abs() const;

    /**
     * Returns the rotational acceleration difference between this angular acceleration
     * and and another angular acceleration.
     *
     * @param other the second angular acceleration.
     *
     * @return the absolute difference in angular acceleration between this angular
     * acceleration acceleration and other
     */
    constexpr AngularAcceleration minDiff(const AngularAcceleration& other);

    // Delete methods that do not apply to angular acceleration
    static constexpr Angle quarter()              = delete;
    static constexpr Angle half()                 = delete;
    static constexpr Angle threeQuarter()         = delete;
    static constexpr Angle full()                 = delete;
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

inline constexpr AngularAcceleration::AngularAcceleration() : Angle(0.0) {}

inline constexpr AngularAcceleration::AngularAcceleration(const Angle& angle)
    : Angle(angle)
{
}

inline constexpr AngularAcceleration AngularAcceleration::zero()
{
    return AngularAcceleration(Angle::zero());
}

inline constexpr AngularAcceleration AngularAcceleration::fromRadians(double rad)
{
    return AngularAcceleration(Angle::fromRadians(rad));
}

inline constexpr AngularAcceleration AngularAcceleration::fromDegrees(double deg)
{
    return AngularAcceleration::fromRadians(deg / 180.0 * M_PI);
}

inline constexpr AngularAcceleration AngularAcceleration::abs() const
{
    return AngularAcceleration::fromRadians(toRadians() < 0 ? -toRadians() : toRadians());
}

inline constexpr AngularAcceleration AngularAcceleration::minDiff(
    const AngularAcceleration& other)
{
    return AngularAcceleration::fromRadians(
        std::fabs(this->toRadians() - other.toRadians()));
}
