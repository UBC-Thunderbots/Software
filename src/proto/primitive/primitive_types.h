#pragma once

#include "software/util/make_enum/make_enum.h"

MAKE_ENUM(AutoChipOrKickMode, AUTOKICK, AUTOCHIP, OFF);

MAKE_ENUM(BallCollisionType, AVOID, ALLOW);

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
