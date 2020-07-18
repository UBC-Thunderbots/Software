#pragma once

#include "software/util/make_enum/make_enum.h"

/**
 * The possible team side of SSL robots
 */
MAKE_ENUM(TeamSide, FRIENDLY, ENEMY);

/**
 * The possible team colours of SSL robots
 */
MAKE_ENUM(TeamColour, YELLOW, BLUE)

/**
 * A light structure for a robot ID with TeamSide
 */
struct RobotIdWithTeamSide
{
    unsigned int id;
    TeamSide team_side;

    bool operator==(const RobotIdWithTeamSide &other) const
    {
        return id == other.id && team_side == other.team_side;
    }

    bool operator<(const RobotIdWithTeamSide &other) const
    {
        return id < other.id || team_side < other.team_side;
    }
};
