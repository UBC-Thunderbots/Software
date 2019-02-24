/**
 * This file contains independent Evaluation functions to evaluate threats to the friendly
 * team on the playing field
 */

#ifndef THUNDERBOTS_ALL_DETECT_THREAT_H
#define THUNDERBOTS_ALL_DETECT_THREAT_H

#include <optional>

#include "ai/world/field.h"
#include "software/ai/world/ball.h"

class BallThreat
{
   public:
    /**
     * Calculates if the ball velocity will intersect with the friendly net.
     *
     * @param ball, the ball object that defines ball position and velocity
     * @param field, the field object that defines friendly goal post locations
     *
     * @return Point containing the intersection of the ball velocity and the friendly
     * net, if the intersection does not exist returns std::nullopt
     */
    static std::optional<Point> calc_ball_vel_intersect_friendly_net(Ball ball,
                                                                     Field field);

    /**
     * Calculates if the ball velocity will intersect with the enemy net.
     *
     * @param ball, the ball object that defines ball position and velocity
     * @param field, the field object that defines enemy goal post locations
     *
     * @return Point containing the intersection of the ball velocity and the enemy net,
     * if the intersection does not exist returns std::nullopt
     */
    static std::optional<Point> calc_ball_vel_intersect_enemy_net(Ball ball, Field field);

   private:
};


#endif  // THUNDERBOTS_ALL_DETECT_THREAT_H
