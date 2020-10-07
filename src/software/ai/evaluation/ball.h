#pragma once

#include "software/world/ball.h"
#include "software/world/field.h"

/**
 * Returns true if the ball is in the friendly half of the field, and false otherwise
 *
 * @param field The field
 * @param ball The ball
 * @return true if the ball is in the friendly half of the field, and false otherwise
 */
bool ballInFriendlyHalf(const Field &field, const Ball &ball);

/**
 * Returns true if the ball is in the enemy half of the field, and false otherwise
 *
 * @param field The field
 * @param ball The ball
 * @return true if the ball is in the enemy's half of the field, and false otherwise
 */
bool ballInEnemyHalf(const Field &field, const Ball &ball);

/**
 * Returns true if the ball is in within the provided radius in one of the friendly
 * corner.
 *
 * @param field The field
 * @param ball The ball
 * @param the radius from the corner, to decide whether or not the ball is in the
 * field.
 */
bool ballInFriendlyCorner(const Field &field, const Ball &ball, double radius);

/**
 * Returns true if the ball is in within the provided radius in one of the enemy
 * corner.
 *
 * @param field The field
 * @param ball The ball
 * @param the radius from the corner, to decide whether or not the ball is in the
 * field.
 */
bool ballInEnemyCorner(const Field &field, const Ball &ball, double radius);

/**
 * Software approximation that finds if a ball has been kicked, regardless of whether the
 * kick was a pass, shot, or chip.
 *
 * @param ball The ball that can be kicked
 * @param expected_kick_direction The direction that we expect the ball to be kicked
 * towards
 * @param min_kick_speed The minimum speed of the ball to be considered a kick, in metres
 * per second
 *
 * @return True if ball was kicked in the approximate direction we expect, false otherwise
 */
bool hasBallBeenKicked(const Ball &ball, const Angle &expected_kick_direction,
                       double min_kick_speed = 0.5);
