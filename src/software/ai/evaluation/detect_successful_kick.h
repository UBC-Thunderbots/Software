#pragma once

#include "software/geom/vector.h"
#include "software/world/ball.h"

/**
 * Software approximation that finds if a ball has been kicked, regardless of whether the kick
 * was a pass, shot, or chip.
 *
 * @param ball The ball that can be kicked
 * @param kick_direction
 */
bool successfulKickDetected(const Ball& ball, const Vector& kick_direction);
