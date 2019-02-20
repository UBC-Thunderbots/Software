#pragma once

/**
 * This enum represents the IDs of the primitives implemented in firmware.
 */
enum class FirmwarePrimitiveType
{
    /**
     * Stop Primitive
     *
     * The parameters are unused.
     *
     * The extra field is 0 for coasting or 1 for braking.
     */
    STOP = 0,

    /**
     * Move Primitive
     *
     * The parameters are the relative position, the relative orientation,
     * and the time delta.
     *
     * The extra field is 0 if the caller doesn’t care about orientation,
     * or 1 if it does.
     */
    MOVE = 1,

    /**
     * Dribble Primitive
     *
     * The parameters are the relative position and orientation.
     *
     * The extra field is 0 if small kicks are prohibited or 1 if they are
     * allowed.
     */
    DRIBBLE = 2,

    /**
     * Shoot Primitive
     *
     * Note that in HL, this is split into kick and chip.
     *
     * The parameters are the relative position, relative orientation, and
     * power (either m/s or m).
     *
     * The extra field has bit 0 clear to kick or set to chip, and bit 1
     * set if the caller cares about orientation.
     */
    SHOOT = 3,

    /**
     * Catch Primitive
     *
     * The parameters are the velocity, dribble speed, and margin.
     */
    CATCH = 4,

    /**
     * Pivot Primitive
     *
     * The parameters are the relative centre point, the swing, and the
     * orientation.
     */
    PIVOT = 5,

    /**
     * Spin Primitive
     *
     * The parameters are the relative position and angular velocity.
     */
    SPIN = 6,

    /**
     * Specifies that direct control is in use and wheels are being
     * driven with individual power levels.
     */
    DIRECT_WHEELS = 7,

    /**
     * Specifies that direct control is in use and robot-relative
     * linear and angular velocities are being sent.
     */
    DIRECT_VELOCITY = 8,
};
