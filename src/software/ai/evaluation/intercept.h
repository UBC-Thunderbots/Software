#pragma once

#include <optional>

#include "software/new_geom/point.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/robot.h"

/**
 * Finds the best place for the given robot to intercept the given ball
 *
 * @param ball The ball to intercept
 * @param field The field on which we want the intercept to occur
 * @param robot The robot that will hopefully intercept the ball
 *
 * @return A pair holding the best place that the robot can move to in order to
 * intercept the ball, and the duration into the future at which the pass would occur,
 *         relative to the timestamp of the robot. If no possible intercept could be
 * found within the field bounds, returns std::nullopt
 */
std::optional<std::pair<Point, Duration>> findBestInterceptForBall(
    TimestampedBallState ball, Field field, TimestampedRobotState robot);

/**
 * Returns the estimated position of the ball at a future time, relative to when the
 * ball was last updated.
 *
 * @param position current position of the ball
 * @param velocity current velocity of the ball
 * @param duration_in_future The relative amount of time in the future
 * at which to predict the ball's position. Value must be >= 0.
 * For example, a value of 1.5 seconds would return the estimated position of the ball
 * 1.5 seconds in the future.
 *
 * @throws std::invalid_argument if the ball is estimating the position with a time
 * from the past
 *
 * @return the estimated position of the ball at the given number of milliseconds
 * in the future. Coordinates are in metres.
 */
Point estimateBallPositionAtFutureTime(Point position, Vector velocity,
                                       const Duration &duration_in_future);
