#pragma once

#include "software/util/make_enum/make_enum.h"

/**
 * The possible team sides of SSL robots
 */
MAKE_ENUM(TeamSide, FRIENDLY, ENEMY);

/**
 * The possible team colours of SSL robots
 */
MAKE_ENUM(TeamColour, YELLOW, BLUE)

/**
 * Indicates which team has possession of the ball
 */
MAKE_ENUM(TeamPossession, FRIENDLY_TEAM, ENEMY_TEAM)
