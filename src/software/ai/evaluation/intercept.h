#pragma once

#include <optional>

#include "software/geom/point.h"
#include "software/world/ball.h"
#include "software/world/field.h"
#include "software/world/robot.h"

struct InterceptionResult{
    InterceptionResult(Point point, Duration duration, double final_speed){
        this->point = point;
        this->duration = duration;
        this->final_speed = final_speed;
    }

    Point point;
    Duration duration;
    double final_speed;
};


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

Point robotPositionToFaceBall(const Point &ball_position,
                              const Angle &face_ball_angle,
                              double additional_offset=0.0);

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
std::optional<InterceptionResult> findBestInterceptForBall(const Ball &ball,
                                                           const Field &field,
                                                           const Robot &robot, bool include_fallback_interceptions=false);
