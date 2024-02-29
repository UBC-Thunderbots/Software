#pragma once

#include "software/world/world.h"

/**
 * Returns the point at which the player should shoot to deflect the ball of an
 * enemy to the outside of the field to get another kick/corner
 * 
 * @param world the world
 * @return the point at which to shoot towards
 */
Point deflectOffEnemyTarget(const World& world);
