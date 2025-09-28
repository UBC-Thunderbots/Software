#pragma once

#include <optional>

#include "software/geom/point.h"
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
std::optional<std::pair<Point, Duration>> findBestInterceptForBall(const Ball &ball,
                                                                   const Field &field,
                                                                   const Robot &robot);


/**
 * Attempts to find a reachable overshoot destination for intercepting the ball,
 * adjusting final speed in steps up to the robot's max speed.
 *
 * @param robot The robot attempting to intercept.
 * @param intercept_position The ideal intercept point without overshoot.
 * @param field The field.
 * @param ball_intercept_time Time the ball will take to reach the base position.
 * @param step_speed Speed increment for each overshoot iteration.
 * @param restrict_to_defense_area Whether to restrict overshoot to the friendly defense
 * area.
 * @return The best reachable intercept point (possibly overshot), or base_position if no
 * improvement.
 */
Point findOvershootInterceptPosition(const Robot &robot, const Point intercept_position,
                                     const Field &field, Duration ball_intercept_time,
                                     double step_speed, bool restrict_to_defense_area);
