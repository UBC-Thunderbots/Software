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
 * Calculates the interception point for intercepting balls
 *
 * @param robot The robot to do the interception
 * @param ball The ball to intercept
 * @param field The field to intercept on
 * @param ball_moving_slow_speed_threshold maximum ball speed considered slow, in m/s
 * @param intercept_position_search_interval distance to nudge interception point each step 
 * during optimization, in metres
 *
 * @return the best interception point
 */
Point findInterceptionPoint(const Robot &robot, const Ball &ball, const Field &field,
                            double ball_moving_slow_speed_threshold   = 0.3,
                            double intercept_position_search_interval = 0.1);

/**
 * Converts the ball position to the robot's position given the direction that the
 * robot faces the ball
 *
 * @param ball_position The ball position
 * @param face_ball_angle The angle to face the ball
 * @param additional_offset Additional offset from facing the ball
 *
 * @return the point that the robot should be positioned to face the ball and dribble
 * the ball
 */
Point robotPositionToFaceBall(const Point &ball_position, const Angle &face_ball_angle,
                              double additional_offset = 0.0);
