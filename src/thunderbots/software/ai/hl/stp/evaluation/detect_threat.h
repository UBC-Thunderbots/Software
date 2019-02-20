//
// Created by evan on 17/02/19.
//

#ifndef THUNDERBOTS_ALL_DETECT_THREAT_H
#define THUNDERBOTS_ALL_DETECT_THREAT_H

#include "software/ai/world/ball.h"
#include "ai/world/field.h"
#include <optional>

class BallThreat {

public:
    static std::optional<Point> calc_ball_vel_intersect_friendly_net(Ball ball, Field field);
    static std::optional<Point> calc_ball_vel_intersect_enemy_net(Ball ball, Field field);

private:

};


#endif //THUNDERBOTS_ALL_DETECT_THREAT_H
