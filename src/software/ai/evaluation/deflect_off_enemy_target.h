#pragma once

#include "software/world/world.h"

namespace Evaluation
{
    /* Returns the point at which the player should shoot to deflect the ball of an
     * enemy
     * to the outside of the field to get another kick/corner
     */
    Point deflect_off_enemy_target(World world);
}  // namespace Evaluation
