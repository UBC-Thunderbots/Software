#pragma once

#include "software/geom/point.h"
#include "software/time/duration.h"

/**
 * Calculates the future position of a moving object
 *
 * @pre The units of the parameters are consistent, e.g. acceleration is in (units of
 * initial_position)/seconds^2
 *
 * @param initial_position The initial position of the object
 * @param initial_velocity The initial velocity of the object in position
 * @param constant_acceleration The constant acceleration applied to the object over time
 * @param duration_in_future Duration into the future
 *
 * @return future position of the object
 */
Point calculateFuturePosition(const Point &initial_position,
                              const Vector &intial_velocity,
                              const Vector &constant_acceleration,
                              const Duration &duration_in_future);

/**
 * Calculates the future velocity of a moving object
 *
 * @pre The units of the parameters are consistent, e.g. acceleration is in (units of
 * initial_velocity)/seconds
 *
 * @param initial_velocity The initial velocity of the object in position
 * @param constant_acceleration The constant acceleration applied to the object over time
 * @param duration_in_future Duration into the future
 *
 * @return future velocity of the object
 */
Vector calculateFutureVelocity(const Vector &intial_velocity,
                               const Vector &constant_acceleration,
                               const Duration &duration_in_future);
