#pragma once
#include <cstdint>

#include "util.h"

/**
 * Flags indicating how robots comply with game rules.
 * Flags are set by the Strategy and examined by the Navigator to determine
 * which potential paths are legal.
 */
enum class MoveFlags : uint64_t
{
    /**
     * No flags.
     */
    NONE = 0x0000,

    /**
     * Do not exit the play area of the field.
     */
    CLIP_PLAY_AREA = 0x0001,

    /**
     * Avoid the ball by 50cm as required by ball-out-of-play rules.
     */
    AVOID_BALL_STOP = 0x0002,

    /**
     * Avoid the ball very slightly, e.g. to orbit before touching the ball at a
     * kickoff or free kick.
     */
    AVOID_BALL_TINY = 0x0004,

    /**
     * Do not enter the friendly defence area.
     */
    AVOID_US_DEFENSE = 0x0008,

    /**
     * Stay at least 20cm outside the enemy defense area as required by
     * ball-entering-play rules.
     */
    AVOID_THEM_DEFENSE = 0x0010,

    /**
     * Do not enter the enemy's field half.
     */
    STAY_OWN_HALF = 0x0020,

    /**
     * Stay more than 40cm outside the penalty mark line as required for
     * non-kickers in penalty kick rules.
     */
    PENALTY_KICK_US = 0x0040,

    /**
     * Stay more than 40cm outside the penalty mark line as required for
     * non-goalies in penalty kick rules.
     */
    PENALTY_KICK_THEM = 0x0080,

    /**
     * Drive carefully instead of quickly.
     */
    CAREFUL = 0x0100,

    /**
     * Avoid the ball by a medium amount, for use in regular navigation.
     */
    AVOID_BALL_MEDIUM = 0x0200,
};

GEN_BITWISE(MoveFlags, uint64_t)

/**
 * The union of all existent flags.
 */
constexpr MoveFlags FLAGS_VALID =
    MoveFlags::CLIP_PLAY_AREA | MoveFlags::AVOID_BALL_STOP | MoveFlags::AVOID_BALL_TINY |
    MoveFlags::AVOID_BALL_MEDIUM | MoveFlags::AVOID_US_DEFENSE |
    MoveFlags::AVOID_THEM_DEFENSE | MoveFlags::STAY_OWN_HALF |
    MoveFlags::PENALTY_KICK_US | MoveFlags::PENALTY_KICK_THEM | MoveFlags::CAREFUL;

/**
 * Checks if a MoveFlag is valid.
 * @param flag MoveFlag to check
 * @return true if valid, false if not
 */
inline bool isMoveFlagValid(MoveFlags flag)
{
    return (flag & ~FLAGS_VALID) == MoveFlags::NONE;
}
