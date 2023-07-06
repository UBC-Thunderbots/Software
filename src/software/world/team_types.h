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
MAKE_ENUM(TeamPossession,
          // Friendly team has possession over ball
          FRIENDLY_TEAM,
          // Enemy team has possession over ball
          ENEMY_TEAM,
          // Enemy possession is stagnant (undisturbed for some time)
          STAGNANT_ENEMY_TEAM)
