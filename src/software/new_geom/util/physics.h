#pragma once

#include "software/new_geom/point.h"

/**
 * Calculates the future position of a moving object
 *
 * @pre The units of the parameters are consistent, e.g. accleration is in (units of
 * initial_position)/(units of time_in_future)^2
 *
 * @param initial_position The initial position of the object
 * @param initial velocity The initial velocity of the object in position
 * @param acceleration The constant acceleration applied to the object over time
 * @param time_in_future The time in the future to estimate position
 *
 * @return future position of the object
 */
Point calculateFuturePosition(const Point &initial_position,
                              const Vector &intial_velocity, const Vector &acceleration,
                              double time_in_future);
