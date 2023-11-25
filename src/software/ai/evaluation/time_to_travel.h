#pragma once

#include "software/time/duration.h"

/**
 * Calculate minimum time it would take for a robot to move a set distance.
 *
 * The calculations are done in 1D to simplify calculations and be as performant as
 * possible.
 * The formulation is made general to work with different units (e.g. meters, radians,
 * etc.). Note that the same units should be used for all parameters.
 *
 * @param distance The distance which the robot is moving. distance is clamped
 * to non-negative numbers.
 * @param max_velocity The maximum linear velocity the robot can travel at (unit/sec).
 * max_velocity is clamped to non-negative numbers.
 * @param max_acceleration The maximum acceleration of the robot (unit/sec^2).
 * max_acceleration is clamped to positive numbers (values greater than 1e-6).
 * @param initial_velocity The initial velocity of the robot (unit/sec). initial_velocity
 * is clamped to [-max_velocity, max_velocity].
 * @param final_velocity The desired final velocity which the robot should be moving at
 * (unit/sec). final_velocity is clamped to [0, max_velocity] since the robot should
 * not be moving back to the start position once it reaches the destination.
 *
 * @return The minimum theoretical time it would take the robot to travel the distance
 * in seconds.
 */
Duration getTimeToTravelDistance(const double distance, const double max_velocity,
                                 const double max_acceleration,
                                 const double initial_velocity = 0,
                                 const double final_velocity   = 0);
