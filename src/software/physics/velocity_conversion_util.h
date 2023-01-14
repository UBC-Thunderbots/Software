#pragma once
#include "software/geom/vector.h"

/**
 * Global velocity refers to the velocity of the robot relative to the field coordinate
 * frame. +x is right, +y is up
 * ↑ +y
 * │
 * │
 * │
 * └───────────► +x
 *
 * Local velocity refers to the velocity of the robot relative to the robot's coordinate
 * frame. +x is forward (towards the dribbler), +y is left The below axis is drawn from
 * the perspective of looking down on the robot from above, with the robot's dribbler
 * facing towards the top of the screen.
 *               ↑ +x
 *               │
 *               │
 *               │
 * +y ◄──────────┘
 */


/**
 * Convert global velocity to local velocity.
 * @param global_vector The global velocity vector
 * @param global_orientation The global orientation of the robot
 * @return The local velocity vector
 */
inline Vector globalToLocalVelocity(const Vector &global_vector,
                                    const Angle &global_orientation)
{
    return global_vector.rotate(-global_orientation);
}

/**
 * Convert local velocity to global velocity.
 * @param local_vector The local velocity vector
 * @param global_orientation The global orientation of the robot
 * @return The global velocity vector
 */
inline Vector localToGlobalVelocity(const Vector &local_vector,
                                    const Angle &global_orientation)
{
    return local_vector.rotate(global_orientation);
}
