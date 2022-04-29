#pragma once

#include "software/time/duration.h"

/**
 * Calculate minimum time it would take for an robot to move a set distance
 *
 * The calculations are done in 1D to simplify calculations and be as performant as
 * possible
 *
 * @param distance The distance which the robot is moving
 * @param max_velocity The maximum linear velocity the robot can travel at (m/s)
 * @param max_acceleration The maximum acceleration of the robot (m/s^2)
 * @param initial_velocity The initial velocity of the robot (m/s)
 * @param final_velocity The desired final velocity which the robot should be moving at
 * (m/s)
 *
 * @return The minimum theoretical time it would take the robot to reach the dest
 * point
 */
Duration getTimeToTravelDistance(const double distance, const double max_velocity,
                                 const double max_acceleration,
                                 const double initial_velocity = 0,
                                 const double final_velocity   = 0);
