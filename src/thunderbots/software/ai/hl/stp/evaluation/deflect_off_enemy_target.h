//
// Created by roark on 02/02/19.
//


#ifndef THUNDERBOTS_ALL_DEFLECT_OFF_ENEMY_TARGET_H
#define THUNDERBOTS_ALL_DEFLECT_OFF_ENEMY_TARGET_H

#include "software/ai/world/field.h"
#include "software/ai/world/world.h"
#include "software/software/geom/line.h"
#include "software/software/geom/util.h"

namespace Evaluation
{
    /* Returns the point at which the player should shoot to deflect the ball of an
     * enemy
     * to the outside of the field to get another kick/corner
     */
    Point deflect_off_enemy_target(World world);
}  // namespace Evaluation

#endif
