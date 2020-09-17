#pragma once

#include "software/geom/vector.h"
#include "software/world/ball.h"

/**
 * Software approximation that finds if a ball has been kicked, regardless of whether the
 * kick was a pass, shot, or chip.
 *
 * @param ball The ball that can be kicked
 * @param kick_direction The direction that we expect the ball to be kicked towards
 *
 * @return True if ball was kicked in the approximate direction we expect, false otherwise
 */
bool successfulKickDetected(const Ball& ball, const Vector& kick_direction);
