#pragma once

#include <optional>

#include "software/world/ball.h"
#include "software/world/field.h"

/**
 * Independent Evaluation functions to evaluate threats to the friendly
 * team on the playing field
 */

/**
 * Calculates if the ball velocity will intersect with the friendly net.
 *
 * @param ball, the ball object that defines ball position and velocity
 * @param field, the field object that defines friendly goal post locations
 *
 * @return Point containing the intersection of the ball velocity and the friendly
 * net, if the intersection does not exist returns std::nullopt
 */
std::optional<Point> calcBallVelIntersectFriendlyNet(Ball ball, Field field);

/**
 * Calculates if the ball velocity will intersect with the enemy net.
 *
 * @param ball, the ball object that defines ball position and velocity
 * @param field, the field object that defines enemy goal post locations
 *
 * @return Point containing the intersection of the ball velocity and the enemy net,
 * if the intersection does not exist returns std::nullopt
 */
std::optional<Point> calcBallVelIntersectEnemyNet(Ball ball, Field field);
