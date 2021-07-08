#pragma once

#include "software/util/make_enum/make_enum.h"

/**
 * OFF - dribbler is off
 * INDEFINITE - the dribbler can be run at this speed indefinitely
 * MAX_FORCE - the dribbler applies maximum force to the ball
 */
MAKE_ENUM(DribblerMode, OFF, INDEFINITE, MAX_FORCE);

MAKE_ENUM(AutoChipOrKickMode, AUTOKICK, AUTOCHIP, OFF);

struct AutoChipOrKick
{
    AutoChipOrKickMode auto_chip_kick_mode;
    union
    {
        double autokick_speed_m_per_s;
        double autochip_distance_m;
    };

    bool operator==(const AutoChipOrKick &other) const
    {
        if (auto_chip_kick_mode == other.auto_chip_kick_mode)
        {
            if (auto_chip_kick_mode != AutoChipOrKickMode::OFF)
            {
                return autokick_speed_m_per_s == other.autokick_speed_m_per_s;
            }
            else
            {
                return true;
            }
        }
        return false;
    }

    bool operator!=(const AutoChipOrKick &other) const
    {
        return !((*this) == other);
    }
};

/**
 * PHYSICAL_LIMIT maximum speed allowed by the physical limits of the robot
 * STOP_COMMAND maximum speed allowed when responding to a stop command
 * TIPTOE maximum speed allowed that does not bump the ball to kick speed
 */
MAKE_ENUM(MaxAllowedSpeedMode, PHYSICAL_LIMIT, STOP_COMMAND, TIPTOE);

