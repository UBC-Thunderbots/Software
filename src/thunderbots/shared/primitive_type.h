#pragma once

/**
 * These represent the IDs of the primitives implemented in firmware.
 *
 */
enum class PrimitiveType
{
    /**
     * Stop primitive.
     * The parameters are unused.
     *
     * The extra field is 0 for coasting or 1 for braking.
     */
    STOP = 0,

    /**
     * \brief Implements the \ref Drive::Robot::move_move family of
     * primitives.
     *
     * The parameters are the relative position, the relative orientation,
     * and the time delta.
     *
     * The extra field is 0 if the caller doesn’t care about orientation,
     * or 1 if it does.
     */
    MOVE = 1,

    /**
     * \brief Implements the \ref Drive::Robot::move_dribble primitive.
     *
     * The parameters are the relative position and orientation.
     *
     * The extra field is 0 if small kicks are prohibited or 1 if they are
     * allowed.
     */
    DRIBBLE = 2,

    /**
     * \brief Implements the \ref Drive::Robot::move_shoot family of
     * primitives.
     *
     * The parameters are the relative position, relative orientation, and
     * power (either m/s or m).
     *
     * The extra field has bit 0 clear to kick or set to chip, and bit 1
     * set if the caller cares about orientation.
     */
    SHOOT = 3,

    /**
     * \brief Implements the \ref Drive::Robot::move_catch primitive.
     *
     * The parameters are the angle difference, the left/right
     * displacement, and the speed.
     */
    CATCH = 4,

    /**
     * \brief Implements the \ref Drive::Robot::move_pivot primitive.
     *
     * The parameters are the relative centre point, the swing, and the
     * orientation.
     */
    PIVOT = 5,

    /**
     * \brief Implements the \ref Drive::Robot::move_spin primitive.
     *
     * The parameters are the relative position and angular velocity.
     */
    SPIN = 6,

    /**
     * \brief Specifies that direct control is in use and wheels are being
     * driven with individual power levels.
     */
    DIRECT_WHEELS = 7,

    /**
     * \brief Specifies that direct control is in use and robot-relative
     * linear and angular velocities are being sent.
     */
    DIRECT_VELOCITY = 8,
};