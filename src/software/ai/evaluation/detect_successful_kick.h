#pragma once

#include "software/geom/vector.h"
#include "software/world/ball.h"

/**
 * Software approximation that finds if a ball has been kicked, regardless of whether the
 * kick was a pass, shot, or chip.
 *
 * @param ball The ball that can be kicked
 * @param expected_kick_direction The direction that we expect the ball to be kicked towards
 * @param min_kick_speed The minimum speed of the ball to be considered a kick
 *
 * @return True if ball was kicked in the approximate direction we expect, false otherwise
 */
bool hasBallBeenKicked(const Ball& ball, const Angle & expected_kick_direction, double min_kick_speed = 0.5);
